// ==================== MacroPlacer.h ====================
#pragma once
#include <map>
#include <set>
#include <vector>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include "odb/db.h"

namespace macroplacer {

struct Rect {
  int x1, y1, x2, y2;
};

struct Cluster {
  int id;
  std::vector<int> cellIndices;
  float centroid_x;
  float centroid_y;
  std::vector<int> connectedClusters;
};

struct Macro {
  int index;
  odb::dbInst* inst;
  int width;
  int height;

  // <<< TRACK‐ALIGNMENT >>>
  int maxPinLocalY = 0;                 // highest pin‐shape y in macro‐local coords
  odb::dbOrientType origOrient;         // remember original orientation
  int origX, origY;                     // remember original placement (optional)
  // <<< /TRACK‐ALIGNMENT >>>
};

class MacroPlacer {
 public:
  MacroPlacer(odb::dbBlock* block);
  void run();

 private:
  int _coreXmin;
  int _coreYmin;
  int _coreWidth;
  int _coreHeight;

  odb::dbBlock* _block;

  std::vector<odb::dbInst*> _cellList;
  std::map<odb::dbInst*, int> _cellMap;
  std::map<int, int> _cellToCluster;
  std::map<int, Cluster> _clusters;

  std::vector<Macro> _macros;
  std::map<int, std::set<int>> _macroToClusters;

  // <<< TRACK‐ALIGNMENT >>>
  // layerName → sorted vector of all horizontal track Y’s on that layer
  std::unordered_map<std::string, std::vector<int>> _horizTracksByLayer;
  // <<< /TRACK‐ALIGNMENT >>>

  std::vector<std::pair<float,float>> anchorCentroids;

  void dumpPinBBoxes();
  void clusterStandardCells();
  void buildMacroClusterAffinity();
  void buildClusterGraph();
  void forceDirectedClusterPlacement();
  void pickThreeAnchors();
  int pickNearestAnchor(int macroIndex) const;
  void placeMacros();

  bool spiralSearch(float cx,
                    float cy,
                    int macroW,
                    int macroH,
                    int pad,
                    const std::vector<Rect>& placedRects,
                    int& final_x,
                    int& final_y,
                    float maxRadius);
};

}  // namespace macroplacer
