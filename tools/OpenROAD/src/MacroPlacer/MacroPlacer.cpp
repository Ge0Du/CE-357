// ==================== MacroPlacer.cpp ====================
#include "MacroPlacer.h"
#include <cmath>
#include <iostream>
#include <queue>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

// ──────────────────────────────────────────────────────────────────────────────
//   Compile‐time constants (adjust as needed)
// ──────────────────────────────────────────────────────────────────────────────
static constexpr int    MAX_CLUSTER_SIZE   = 1000;  // or whatever limit you prefer
static constexpr int    MACRO_SAFETY_PAD   = 0;     // pad around each macro
static constexpr float  STD_CELL_SPACING   = 1.0f;  // spacing factor for StdCell → margin
// ──────────────────────────────────────────────────────────────────────────────

namespace macroplacer {

MacroPlacer::MacroPlacer(odb::dbBlock* block)
    : _block(block)
{
  odb::Rect core = _block->getCoreArea();
  _coreXmin   = core.xMin();
  _coreYmin   = core.yMin();
  _coreWidth  = core.xMax() - core.xMin();
  _coreHeight = core.yMax() - core.yMin();

  // <<< TRACK‐ALIGNMENT >>>
  // 1) Extract all horizontal track Y’s for each layer (we’ll use "M2" by default)
  {
    odb::dbTech* tech = block->getDb()->getTech();
    for (odb::dbTrackGrid* tg = tech->getTrackGrid(); tg; tg = tg->getNext()) {
      if (tg->getDirection() != odb::dbTrackDir::HORIZONTAL) continue;
      odb::dbTechLayer* layer = tg->getLayer();
      std::string layerName = layer->getName();  // e.g. "M2", "M3", …

      int start  = tg->getStartTrack();
      int space  = tg->getTrackSpacing();
      int count  = tg->getNumTracks();

      auto &vec = _horizTracksByLayer[layerName];
      vec.reserve(count);
      for (int i = 0; i < count; ++i) {
        vec.push_back(start + i * space);
      }
      std::sort(vec.begin(), vec.end());
    }
  }
  // <<< /TRACK‐ALIGNMENT >>>

  int cell_index = 0;
  for (auto* inst : block->getInsts()) {
    if (inst->getMaster()->isBlock()) {
      // Build Macro struct, including maxPinLocalY
      Macro m{
        static_cast<int>(_macros.size()),
        inst,
        static_cast<int>(inst->getBBox()->getDX()),
        static_cast<int>(inst->getBBox()->getDY())
      };

      // <<< TRACK‐ALIGNMENT >>>
      // 2) Compute the maximum pin‐Y in *macro‐local* coordinates
      int maxPinY = 0;
      for (auto* iterm : inst->getITerms()) {
        for (auto* shape = iterm->getMTerm()->getLayerPinShapes(); shape; shape = shape->getNext()) {
          maxPinY = std::max(maxPinY, shape->yMax());
        }
      }
      m.maxPinLocalY = maxPinY;

      // Record original orientation/position (optional)
      m.origOrient = inst->getOrient();
      odb::Point origPt = inst->getLocation();
      m.origX = origPt.x();
      m.origY = origPt.y();
      // <<< /TRACK‐ALIGNMENT >>>

      _macros.push_back(m);
    } else {
      _cellMap[inst] = cell_index;
      _cellList.push_back(inst);
      ++cell_index;
    }
  }
}

void MacroPlacer::dumpPinBBoxes()
{
  std::cout << "=== Dumping pin bounding boxes ===" << std::endl;
  for (auto& m : _macros) {
    auto* inst = m.inst;
    std::cout << "Inst: " << inst->getName() << std::endl;
    for (auto* iterm : inst->getITerms()) {
      odb::Rect bbox = iterm->getBBox();
      int x1 = bbox.xMin();
      int y1 = bbox.yMin();
      int x2 = bbox.xMax();
      int y2 = bbox.yMax();
      std::cout << "  Pin " << iterm->getMTerm()->getName()
                << "  bbox=(" << x1 << "," << y1 << ")-(" << x2 << "," << y2 << ")"
                << std::endl;
    }
  }
  std::cout << std::endl;
}

void MacroPlacer::run()
{
  // DEBUG: dump initial pin geometry
  dumpPinBBoxes();

  clusterStandardCells();
  buildMacroClusterAffinity();
  buildClusterGraph();
  forceDirectedClusterPlacement();

  // ── NEW ── pick three “sparse” anchors after force‐directed placement
  pickThreeAnchors();

  placeMacros();

  // DEBUG: dump pin geometry after placement
  dumpPinBBoxes();
}

void MacroPlacer::clusterStandardCells()
{
  std::vector<std::vector<int>> graph(_cellList.size());
  for (auto* net : _block->getNets()) {
    std::vector<int> indices;
    for (auto* it : net->getITerms()) {
      if (auto* inst = it->getInst(); inst && _cellMap.count(inst)) {
        indices.push_back(_cellMap[inst]);
      }
    }
    for (size_t i = 0; i < indices.size(); ++i) {
      for (size_t j = i + 1; j < indices.size(); ++j) {
        graph[indices[i]].push_back(indices[j]);
        graph[indices[j]].push_back(indices[i]);
      }
    }
  }

  std::vector<bool> visited(_cellList.size(), false);
  int cluster_id = 0;
  for (size_t i = 0; i < _cellList.size(); ++i) {
    if (visited[i]) continue;
    std::queue<int> q;
    q.push(i);
    visited[i] = true;
    _clusters[cluster_id].id = cluster_id;
    int count = 0;
    while (!q.empty() && count < MAX_CLUSTER_SIZE) {
      int u = q.front();
      q.pop();
      auto& cl = _clusters[cluster_id];
      cl.cellIndices.push_back(u);
      _cellToCluster[u] = cluster_id;
      ++count;
      for (int v : graph[u]) {
        if (!visited[v]) {
          visited[v] = true;
          q.push(v);
        }
      }
    }
    ++cluster_id;
  }

  odb::Rect core = _block->getCoreArea();
  float midX = 0.5f * (core.xMin() + core.xMax());
  float midY = 0.5f * (core.yMin() + core.yMax());
  for (auto& [id, cl] : _clusters) {
    float sx = 0, sy = 0;
    for (int idx : cl.cellIndices) {
      auto* inst = _cellList[idx];
      if (inst->isPlaced()) {
        sx += inst->getBBox()->xMin();
        sy += inst->getBBox()->yMin();
      } else {
        sx += midX;
        sy += midY;
      }
    }
    if (!cl.cellIndices.empty()) {
      cl.centroid_x = sx / cl.cellIndices.size();
      cl.centroid_y = sy / cl.cellIndices.size();
    }
  }

  std::mt19937 rng(42);
  std::uniform_real_distribution<float> jitterX(-_coreWidth * 0.05f, _coreWidth * 0.05f);
  std::uniform_real_distribution<float> jitterY(-_coreHeight * 0.05f, _coreHeight * 0.05f);
  for (auto& [id, cl] : _clusters) {
    if (!cl.cellIndices.empty()) {
      cl.centroid_x += jitterX(rng);
      cl.centroid_y += jitterY(rng);
    }
  }

  std::cout << "=== Clusters Formed ===\n";
  for (auto& [id, cl] : _clusters) {
    std::cout << "Cluster " << id
              << " size=" << cl.cellIndices.size()
              << " @(" << cl.centroid_x << "," << cl.centroid_y << ")\n";
  }
}

void MacroPlacer::buildMacroClusterAffinity()
{
  for (auto& m : _macros) {
    for (auto* it : m.inst->getITerms()) {
      if (auto* net = it->getNet()) {
        for (auto* term : net->getITerms()) {
          if (auto* inst = term->getInst(); inst && _cellMap.count(inst)) {
            int cid = _cellToCluster[_cellMap[inst]];
            _macroToClusters[m.index].insert(cid);
          }
        }
      }
    }
  }
}

void MacroPlacer::buildClusterGraph()
{
  for (auto& [id, cl] : _clusters) {
    cl.connectedClusters.clear();
  }

  std::unordered_map<odb::dbNet*, std::unordered_set<int>> netMap;
  for (auto* net : _block->getNets()) {
    std::unordered_set<int> s;
    for (auto* it : net->getITerms()) {
      if (auto* inst = it->getInst(); inst && _cellMap.count(inst)) {
        s.insert(_cellToCluster[_cellMap[inst]]);
      }
    }
    if (s.size() > 1) {
      netMap[net] = s;
    }
  }

  for (auto& [net, s] : netMap) {
    for (int c1 : s) {
      for (int c2 : s) {
        if (c1 != c2) {
          _clusters[c1].connectedClusters.push_back(c2);
        }
      }
    }
  }
}

void MacroPlacer::forceDirectedClusterPlacement()
{
  std::vector<std::pair<float,float>> pins;
  int Npins = 36;
  float cx = _coreXmin + 0.5f * _coreWidth;
  float cy = _coreYmin + 0.5f * _coreHeight;
  float r  = 0.5f * std::min(_coreWidth, _coreHeight);

  for (int i = 0; i < Npins; ++i) {
    float theta = 2.0f * M_PI * i / Npins;
    pins.emplace_back(cx + r * std::cos(theta),
                      cy + r * std::sin(theta));
  }

  const int numIter = 100;
  const float kAttr = 0.001f;
  const float kRep  = 5000.0f;
  const float kPin  = 2000.0f;

  for (int iter = 0; iter < numIter; ++iter) {
    std::unordered_map<int,std::pair<float,float>> F;
    for (auto& [id, cl] : _clusters) {
      F[id] = {0, 0};
    }

    // Repulsive forces
    for (auto& [i, c1] : _clusters) {
      for (auto& [j, c2] : _clusters) {
        if (i == j) continue;
        float dx = c1.centroid_x - c2.centroid_x;
        float dy = c1.centroid_y - c2.centroid_y;
        float d2 = dx * dx + dy * dy + 1e-6f;
        float f  = kRep / d2;
        float d  = std::sqrt(d2);
        F[i].first  += dx/d * f;
        F[i].second += dy/d * f;
      }
    }

    // Attractive (spring) forces between connected clusters
    for (auto& [id, cl] : _clusters) {
      for (int nb : cl.connectedClusters) {
        auto& c2 = _clusters[nb];
        float dx = c2.centroid_x - cl.centroid_x;
        float dy = c2.centroid_y - cl.centroid_y;
        float d  = std::sqrt(dx * dx + dy * dy) + 1e-6f;
        float f  = kAttr * d;
        F[id].first  += dx/d * f;
        F[id].second += dy/d * f;
      }
    }

    // Pin‐circle “pull” (on boundary)
    for (auto& [id, cl] : _clusters) {
      float fx = 0, fy = 0;
      for (auto& [px, py] : pins) {
        float dx = cl.centroid_x - px;
        float dy = cl.centroid_y - py;
        float d2 = dx * dx + dy * dy + 1e-6f;
        float f  = kPin / d2;
        float d  = std::sqrt(d2);
        fx += dx/d * f;
        fy += dy/d * f;
      }
      F[id].first  += fx;
      F[id].second += fy;
    }

    // Update positions (clamp to core)
    for (auto& [id, cl] : _clusters) {
      cl.centroid_x += F[id].first;
      cl.centroid_y += F[id].second;
      cl.centroid_x = std::clamp(cl.centroid_x,
                                 (float)_coreXmin,
                                 (float)(_coreXmin + _coreWidth));
      cl.centroid_y = std::clamp(cl.centroid_y,
                                 (float)_coreYmin,
                                 (float)(_coreYmin + _coreHeight));
    }
  }

  std::cout << "=== Clusters after force‐directed placement ===\n";
  for (auto& [id, cl] : _clusters) {
    std::cout << "Cluster " << id
              << " @(" << cl.centroid_x << "," << cl.centroid_y << ")\n";
  }
}

void MacroPlacer::pickThreeAnchors()
{
  // Build a vector of (cluster_id, size)
  std::vector<std::pair<int,int>> order;
  order.reserve(_clusters.size());
  for (auto& [id, cl] : _clusters) {
    order.emplace_back(id, cl.cellIndices.size());
  }

  // Sort descending by cluster‐size
  std::sort(order.begin(), order.end(),
            [](auto& A, auto& B) {
              return A.second > B.second;
            });

  anchorCentroids.clear();
  anchorCentroids.resize(3);

  if (order.empty()) {
    // No clusters: fallback to core‐center for all three
    float cx = _coreXmin + 0.5f * _coreWidth;
    float cy = _coreYmin + 0.5f * _coreHeight;
    for (int i = 0; i < 3; ++i) {
      anchorCentroids[i] = {cx, cy};
    }
    return;
  }

  // Use the three largest clusters if available
  for (int i = 0; i < 3; ++i) {
    if (i < (int)order.size()) {
      int cid = order[i].first;
      anchorCentroids[i] = { _clusters[cid].centroid_x,
                             _clusters[cid].centroid_y };
    } else {
      // If fewer than 3 clusters exist, duplicate core center
      float cx = _coreXmin + 0.5f * _coreWidth;
      float cy = _coreYmin + 0.5f * _coreHeight;
      anchorCentroids[i] = {cx, cy};
    }
  }

  std::cout << "=== Three placement anchors ===\n";
  for (int i = 0; i < 3; ++i) {
    std::cout << "Anchor " << i
              << " @(" << anchorCentroids[i].first
              << "," << anchorCentroids[i].second << ")\n";
  }
}

int MacroPlacer::pickNearestAnchor(int macroIndex) const
{
  float sumX = 0, sumY = 0;
  int totalCells = 0;

  for (int cid : _macroToClusters.at(macroIndex)) {
    const Cluster& cl = _clusters.at(cid);
    int sz = cl.cellIndices.size();
    sumX += cl.centroid_x * sz;
    sumY += cl.centroid_y * sz;
    totalCells += sz;
  }

  float cx, cy;
  if (totalCells > 0) {
    cx = sumX / totalCells;
    cy = sumY / totalCells;
  } else {
    // No connected StdCell clusters: default to chip center
    cx = _coreXmin + 0.5f * _coreWidth;
    cy = _coreYmin + 0.5f * _coreHeight;
  }

  // Find which of the three anchors is closest to (cx,cy)
  int bestIdx = 0;
  float bestDist2 = std::numeric_limits<float>::infinity();
  for (int i = 0; i < 3; ++i) {
    float dx = cx - anchorCentroids[i].first;
    float dy = cy - anchorCentroids[i].second;
    float d2 = dx*dx + dy*dy;
    if (d2 < bestDist2) {
      bestDist2 = d2;
      bestIdx = i;
    }
  }
  return bestIdx;
}

void MacroPlacer::placeMacros()
{
  // 1) Sort macros by connectivity “weight” (unchanged)
  std::vector<Macro*> order;
  for (auto& m : _macros) {
    order.push_back(&m);
  }
  std::sort(order.begin(), order.end(),
            [&](Macro* a, Macro* b) {
              int ca = 0, cb = 0;
              for (int cid : _macroToClusters[a->index]) {
                ca += _clusters[cid].cellIndices.size();
              }
              for (int cid : _macroToClusters[b->index]) {
                cb += _clusters[cid].cellIndices.size();
              }
              return ca > cb;
            });

  // 2) Gather all row Y‐coordinates and the site width
  auto rows = _block->getRows();
  if (rows.empty()) {
    std::cerr << "Error: No rows defined in block\n";
    return;
  }

  // Assume all rows use the same site; pick the first row to get siteW
  odb::dbRow* first_row = *rows.begin();
  odb::dbSite* site    = first_row->getSite();
  if (!site) {
    std::cerr << "Error: No site defined in block rows\n";
    return;
  }
  int siteW = site->getWidth();

  // Build a vector of every row’s Y using getBBox().yMin()
  std::vector<int> rowYs;
  rowYs.reserve(rows.size());
  for (auto* r : rows) {
    rowYs.push_back(r->getBBox().yMin());
  }
  std::sort(rowYs.begin(), rowYs.end());

  // <<< TRACK‐ALIGNMENT >>>
  // Grab the sorted list of M2 track Y’s
  auto &m2TrackYs = _horizTracksByLayer["M2"];
  // <<< /TRACK‐ALIGNMENT >>>

  // 3) Main placement loop
  std::vector<Rect> placedRects;
  placedRects.reserve(_macros.size());

  for (auto* mp : order) {
    Macro& m = *mp;

    // ── NEW: pick the nearest of our three anchors for this macro
    int anchorId = pickNearestAnchor(m.index);
    float cx = anchorCentroids[anchorId].first;
    float cy = anchorCentroids[anchorId].second;

    // 3.2) Compute “search radius” based on connected StdCells + pad
    int totalCells = 0;
    for (int cid : _macroToClusters[m.index]) {
      totalCells += _clusters[cid].cellIndices.size();
    }
    int pad = MACRO_SAFETY_PAD;
    float margin = std::sqrt((float)totalCells) * STD_CELL_SPACING + pad;
    float searchRadius = margin;

    // 3.3) First attempt: spiralSearch using (macroW,macroH) + margin
    int sx = 0, sy_raw = 0;
    bool placed = spiralSearch(cx,
                               cy,
                               m.width,
                               m.height,
                               pad,
                               placedRects,
                               sx,
                               sy_raw,
                               searchRadius);
    if (!placed) {
      // Second attempt: allow full‐chip search (radius = half of smaller core dimension)
      float fullRadius = 0.5f * std::min(_coreWidth, _coreHeight);
      placed = spiralSearch(cx,
                            cy,
                            m.width,
                            m.height,
                            pad,
                            placedRects,
                            sx,
                            sy_raw,
                            fullRadius);
    }
    if (!placed) {
      std::cerr << "Failed to place " << m.inst->getName() << "\n";
      continue;
    }

    // ── SNAP X TO SITE GRID ──
    sx = ((sx - _coreXmin) / siteW) * siteW + _coreXmin;

    // ── SNAP Y TO “TRACK‐ALIGNED” ROW ──
    int bestRowY = rowYs[0];
    int bestError = INT_MAX;
    for (int ry : rowYs) {
      // If macro bottom = ry, then topmost pin sits at (ry + maxPinLocalY)
      int pinAbsY = ry + m.maxPinLocalY;

      // Find closest M2 track
      auto it = std::lower_bound(m2TrackYs.begin(), m2TrackYs.end(), pinAbsY);
      int cand1 = (it == m2TrackYs.end()   ? m2TrackYs.back() : *it);
      int cand2 = (it == m2TrackYs.begin() ? m2TrackYs.front() : *std::prev(it));
      int err1 = std::abs(pinAbsY - cand1);
      int err2 = std::abs(pinAbsY - cand2);
      int err  = std::min(err1, err2);

      if (err < bestError) {
        bestError = err;
        bestRowY  = ry;
      }
    }
    int sy = bestRowY;

    // ── SANITY CHECK: does the snapped bounding box lie inside the core? ──
    {
      odb::Rect newBBox(sx,
                        sy,
                        sx + m.width - 1,
                        sy + m.height - 1);
      if ( newBBox.xMin() < _coreXmin ||
           newBBox.yMin() < _coreYmin ||
           newBBox.xMax() > _coreXmin + _coreWidth ||
           newBBox.yMax() > _coreYmin + _coreHeight ) {
        std::cerr << "[Error] After snapping, macro "
                  << m.inst->getName()
                  << " out of core: ("
                  << newBBox.xMin() << "," << newBBox.yMin()
                  << ")-("
                  << newBBox.xMax() << "," << newBBox.yMax()
                  << ")\n";
      }
    }

    // 3.4) Place the macro in the DB
    m.inst->setLocation(sx, sy);
    m.inst->setOrient(m.origOrient);  // preserve its original orientation
    m.inst->setPlacementStatus(odb::dbPlacementStatus::FIRM);

    // 3.5) Record its padded bounding‐box for future overlap checks
    placedRects.push_back(Rect{
      sx - pad,
      sy - pad,
      sx + m.width + pad,
      sy + m.height + pad
    });
  }

  std::cout << "Macro placement complete.\n";
}

bool MacroPlacer::spiralSearch(float cx,
                               float cy,
                               int macroW,
                               int macroH,
                               int pad,
                               const std::vector<Rect>& placedRects,
                               int& final_x,
                               int& final_y,
                               float maxRadius)
{
  const float dr    = 100.0f;
  const int   steps = 36;

  float r = 0;
  while (r <= maxRadius) {
    for (int i = 0; i < steps; ++i) {
      float theta = i * 2.0f * M_PI / steps;
      float dx = r * std::cos(theta);
      float dy = r * std::sin(theta);

      // Center of macro = (_coreXmin + cx + dx, _coreYmin + cy + dy)
      int x = int(_coreXmin + cx + dx - 0.5f * macroW);
      int y = int(_coreYmin + cy + dy - 0.5f * macroH);

      Rect cand{
        x - pad,
        y - pad,
        x + macroW + pad,
        y + macroH + pad
      };

      // If cand goes out of core, skip
      if (cand.x1 < _coreXmin ||
          cand.y1 < _coreYmin ||
          cand.x2 > _coreXmin + _coreWidth ||
          cand.y2 > _coreYmin + _coreHeight) {
        continue;
      }

      // If cand overlaps any placedRects, skip
      bool conflict = false;
      for (auto& pr : placedRects) {
        if (!(cand.x2 < pr.x1 ||
              cand.x1 > pr.x2 ||
              cand.y2 < pr.y1 ||
              cand.y1 > pr.y2)) {
          conflict = true;
          break;
        }
      }
      if (conflict) continue;

      // Found a valid spot
      final_x = x;
      final_y = y;
      return true;
    }
    r += dr;
  }
  return false;
}

}  // namespace macroplacer
