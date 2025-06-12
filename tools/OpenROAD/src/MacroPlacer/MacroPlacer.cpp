// ----------------------------------------------
// File: MacroPlacer.cpp
// ----------------------------------------------
#include "MacroPlacer.h"
#include <iostream>
#include <cassert>          // for assert(...)
#include <algorithm>
#include <functional>
#include <stdexcept>

/************************************************************************
 *                       Constructor                                     *
 ************************************************************************/
MacroPlacer::MacroPlacer(std::vector<Block>& blocks,
                         const std::vector<Net>& nets)
  : _blocks(blocks),
    _nets(nets),
    _rng(std::random_device{}()),
    _uniform01(0.0, 1.0)
{
    // 1) Create one TreeNode* leaf for each block
    _allLeaves.clear();
    for (auto &b : _blocks) {
        TreeNode* leaf = new TreeNode(&b);
        _allLeaves.push_back(leaf);
    }

    // 2) Build an initial random slicing tree
    buildInitialSlicingTree();

    // 3) Initialize temperatures to the cost of the initial placement
    double initArea = computeAreaCost();
    double initWire = computeWireCost();
    _T_area = initArea;
    _T_wire = initWire;

    // 4) Set cooling rates, inner‐iteration counts, and stopping criteria
    _alphaA = 0.85;      // area cools by 15% per outer iteration
    _alphaW = 0.90;      // wire cools by 10% per outer iteration
    _innerIters = 1000;  // trials per temperature
    _minTemp = 1e-3;     // stop once both temps < this

    // 5) Save the initial “best” configuration
    _bestCostArea = initArea;
    _bestCostWire = initWire;
    _bestRoot = cloneTree(_root);

    // Reindex leaves/internals for the clone
    _allInternals.clear();
    _allLeaves.clear();
    indexTreeNodes(_bestRoot);
    if (_allInternals.empty()) {
        std::cerr << "[ERROR] No internals after indexing best‐tree!\n";
        std::abort();
    }

    std::cout << "[MacroPlacer] Initialized: AreaCost=" << initArea
              << "  WireCost=" << initWire << "\n";
}

/************************************************************************
 *                       Destructor                                     *
 *  (Already declared/defined in MacroPlacer.h; do NOT redefine here.) *
 ************************************************************************/
// NO destructor implementation here—remove the duplicate definition.

/************************************************************************
 *                       destroyTree                                     *
 ************************************************************************/
void MacroPlacer::destroyTree(TreeNode* n)
{
    if (!n) return;
    destroyTree(n->left);
    destroyTree(n->right);
    delete n;
}

/************************************************************************
 *                  Building the Initial Slicing Tree                   *
 ************************************************************************/
void MacroPlacer::buildInitialSlicingTree()
{
    // We will combine the existing _allLeaves (size N) into a random tree.
    std::vector<TreeNode*> pool = _allLeaves;

    // shuffle pool for randomness
    std::shuffle(pool.begin(), pool.end(), _rng);

    while (pool.size() > 1) {
        // pick two random distinct indices from pool
        std::uniform_int_distribution<int> dist(0, int(pool.size() - 1));
        int i = dist(_rng);
        int j = dist(_rng);
        while (j == i) {
            j = dist(_rng);
        }
        if (i > j) std::swap(i, j);

        TreeNode* A = pool[i];
        TreeNode* B = pool[j];
        assert(A != nullptr && B != nullptr);

        pool.erase(pool.begin() + j);
        pool.erase(pool.begin() + i);

        // random cut type
        bool cutH = (_uniform01(_rng) < 0.5);

        TreeNode* parent = new TreeNode(A, B, cutH);
        pool.push_back(parent);
    }

    assert(pool.size() == 1);
    _root = pool[0];

    // Fully reindex both arrays now that the tree is built
    _allInternals.clear();
    _allLeaves.clear();
    indexTreeNodes(_root);

    if (_allLeaves.empty() || _allInternals.empty()) {
        std::cerr << "[ERROR] Invalid initial slicing tree: "
                  << _allLeaves.size() << " leaves, "
                  << _allInternals.size() << " internals.\n";
        debugPrintTree(_root, 0);
        std::abort();
    }
}

void MacroPlacer::indexTreeNodes(TreeNode* node)
{
    if (!node) return;

    if (node->leafBlock) {
        // Leaf – record it
        _allLeaves.push_back(node);
    }
    else {
        // Internal – record, then recurse
        _allInternals.push_back(node);
        indexTreeNodes(node->left);
        indexTreeNodes(node->right);
    }
}

/************************************************************************
 *                         Cost Computation                             *
 ************************************************************************/
/// Recursively pack the tree rooted at `node`.  Place each leaf at (x,y).
/// Returns (width, height) of that subtree.
std::pair<double,double> MacroPlacer::dfsPack(TreeNode* node,
                                               double originX,
                                               double originY)
{
    if (!node) return {0.0, 0.0};

    if (node->leafBlock) {
        // This is a leaf: place the block at (originX, originY)
        Block* b = node->leafBlock;
        double w = b->placedWidth();
        double h = b->placedHeight();
        b->x = originX;
        b->y = originY;
        node->width = w;
        node->height = h;
        return {w, h};
    }

    // pack left subtree
    auto [wL, hL] = dfsPack(node->left, originX, originY);

    // pack right subtree depending on H or V cut
    auto [wR, hR] = dfsPack(
      node->right,
      node->cutIsHorizontal ? originX : (originX + wL),
      node->cutIsHorizontal ? (originY + hL) : originY
    );

    double W, H;
    if (node->cutIsHorizontal) {
        // children stacked vertically
        W = std::max(wL, wR);
        H = hL + hR;
    } else {
        // children side-by-side
        W = wL + wR;
        H = std::max(hL, hR);
    }
    node->width  = W;
    node->height = H;
    return {W, H};
}

/// Packs the entire tree at (0,0).  Returns bounding-box area = W * H.
double MacroPlacer::packTree(TreeNode* node, double originX, double originY)
{
    auto [W, H] = dfsPack(node, originX, originY);
    return W * H;
}

double MacroPlacer::computeAreaCost()
{
    // Pack the tree at origin (0,0)
    double area = packTree(_root, 0.0, 0.0);
    return area;
}

double MacroPlacer::computeWireCost()
{
    double totalHPWL = 0.0;
    for (auto &net : _nets) {
        if (net.blockIDs.empty()) continue;
        double xMin = std::numeric_limits<double>::infinity();
        double xMax = -std::numeric_limits<double>::infinity();
        double yMin =  xMin;
        double yMax =  xMax;

        for (int bid : net.blockIDs) {
            Block& b = _blocks[bid];
            double cx = b.cx();
            double cy = b.cy();
            xMin = std::min(xMin, cx);
            xMax = std::max(xMax, cx);
            yMin = std::min(yMin, cy);
            yMax = std::max(yMax, cy);
        }
        double hpwl = (xMax - xMin) + (yMax - yMin);
        totalHPWL += hpwl * net.weight;
    }
    return totalHPWL;
}

/************************************************************************
 *                         Move Operations                              *
 ************************************************************************/
/// (1) Swap the leafBlock pointers of two randomly chosen leaves.
///    Does not change tree structure, only swaps which Block sits at which leaf.
bool MacroPlacer::moveSwapTwoLeaves()
{
    if (_allLeaves.size() < 2) return false;

    // Sanity check: ensure no null entries
    for (auto ptr : _allLeaves) {
        if (ptr == nullptr || ptr->leafBlock == nullptr) {
            std::cerr << "[moveSwapTwoLeaves] Found invalid leaf pointer or missing leafBlock.\n";
            std::abort();
        }
    }

    std::uniform_int_distribution<int> dist(0, int(_allLeaves.size() - 1));
    int i = dist(_rng);
    int j = dist(_rng);
    while (j == i) j = dist(_rng);

    TreeNode* A = _allLeaves[i];
    TreeNode* B = _allLeaves[j];
    assert(A->leafBlock && B->leafBlock);

    // Save old state (in case we want to roll back)
    Block* oldA = A->leafBlock;
    Block* oldB = B->leafBlock;
    bool   oldRotA = oldA->rotated;
    bool   oldRotB = oldB->rotated;

    // Swap the pointers:
    A->leafBlock = oldB;
    B->leafBlock = oldA;

    // Swap each block’s “rotated” bit so leaf‐node orientation is preserved
    std::swap(oldA->rotated, oldB->rotated);

    return true;
}

/// (2) Cut a leaf from the tree and re-insert it somewhere else.
///     - Pick a random leaf L.
///     - Remove L’s parent by splicing in L’s sibling.
///     - Pick a random internal node I (or root) to re-insert under.
///     - Create a new internal node N that has (L, oldChild) as children.
///     - Reindex internals & leaves afterwards.
bool MacroPlacer::moveCutAndReinsertLeaf()
{
    // Pre-move sanity checks
    if (_allLeaves.size() < 2 || _allInternals.empty()) return false;

    // 1) Pick a random leaf
    std::uniform_int_distribution<int> leafDist(0, int(_allLeaves.size() - 1));
    int idxL = leafDist(_rng);
    TreeNode* leafNode = _allLeaves[idxL];
    assert(leafNode && leafNode->leafBlock);

    // 2) Find parent of leafNode
    std::function<TreeNode*(TreeNode*, TreeNode*)> findParent =
      [&](TreeNode* cur, TreeNode* child) -> TreeNode* {
          if (!cur || cur->leafBlock) return nullptr;
          if (cur->left == child || cur->right == child) return cur;
          TreeNode* res = findParent(cur->left, child);
          if (res) return res;
          return findParent(cur->right, child);
      };

    TreeNode* parent = findParent(_root, leafNode);
    if (!parent) {
        // leafNode was root → cannot reinsert
        return false;
    }

    // 3) Determine sibling
    TreeNode* sibling = (parent->left == leafNode ? parent->right : parent->left);
    assert(sibling);

    // 4) Splice `parent` out by hooking `sibling` into grandparent
    TreeNode* grandparent = findParent(_root, parent);
    if (!grandparent) {
        // parent was root → sibling becomes new root
        _root = sibling;
    } else {
        if (grandparent->left == parent) {
            grandparent->left = sibling;
        } else {
            grandparent->right = sibling;
        }
    }

    // Save parent's state in case we need to roll back
    parent->savedChildren = { parent->left, parent->right };
    parent->savedCutType = parent->cutIsHorizontal;

    // 5) Immediately re-index _allInternals and _allLeaves after splicing out
    _allInternals.clear();
    _allLeaves.clear();
    indexTreeNodes(_root);

    // If after splicing, the tree collapsed into a single leaf (or has no internals),
    // then we cannot reinsert—just abort this move.
    if (_allInternals.empty()) {
        // Restore original parent pointers (rollback)
        // Note: in practice you might want a more thorough rollback, but
        // for simplicity we'll just skip this move entirely.
        // (Parent is still in memory but not in the tree; we leave it alone.)
        return false;
    }

    // 6) Pick a random internal node (freshly rebuilt)
    std::uniform_int_distribution<int> intDist(0, int(_allInternals.size() - 1));
    TreeNode* chosen = _allInternals[intDist(_rng)];
    assert(chosen && chosen->leafBlock == nullptr);

    // 7) Decide whether to insert as left or right
    bool asLeft = (_uniform01(_rng) < 0.5);

    // Save leaf’s rotation in case we need to roll back
    Block* oldLeafBlock = leafNode->leafBlock;
    bool oldRot = oldLeafBlock->rotated;

    // We will splice `leafNode` under `chosen`.  Save chosen’s state, too.
    chosen->savedChildren = { chosen->left, chosen->right };
    chosen->savedCutType = chosen->cutIsHorizontal;

    // Pick which child of `chosen` to push down
    TreeNode* childToSplice = (asLeft ? chosen->left : chosen->right);
    if (!childToSplice) {
        // Should never happen if chosen is a true internal
        return false;
    }

    // Create a brand-new internal node
    TreeNode* newInternal = new TreeNode(nullptr, nullptr,
                                         (_uniform01(_rng) < 0.5));

    if (asLeft) {
        newInternal->left = leafNode;
        newInternal->right = childToSplice;
        chosen->left = newInternal;
    } else {
        newInternal->left = childToSplice;
        newInternal->right = leafNode;
        chosen->right = newInternal;
    }

    // 8) Rebuild internals/leaves once more
    _allInternals.clear();
    _allLeaves.clear();
    indexTreeNodes(_root);

    // Final sanity check: if somehow the tree is invalid, signal failure
    if (_allInternals.empty() || _allLeaves.size() < 2) {
        // Tree collapsed in a bad way—abort
        return false;
    }

    return true;
}

/// (3) Flip the rotated bit of one randomly chosen leaf
bool MacroPlacer::moveRotateLeaf()
{
    if (_allLeaves.empty()) return false;

    // Sanity check:
    for (auto ptr : _allLeaves) {
        if (!ptr || !ptr->leafBlock) {
            std::cerr << "[moveRotateLeaf] Found invalid leaf or missing leafBlock.\n";
            std::abort();
        }
    }

    std::uniform_int_distribution<int> dist(0, int(_allLeaves.size() - 1));
    int idx = dist(_rng);
    TreeNode* leaf = _allLeaves[idx];
    assert(leaf->leafBlock);

    Block* b = leaf->leafBlock;
    b->rotated = !b->rotated;
    return true;
}

/// (4) Flip the cut type (H ↔ V) of a randomly chosen internal node
bool MacroPlacer::moveFlipCut()
{
    if (_allInternals.empty()) return false;

    // Sanity check:
    for (auto ptr : _allInternals) {
        if (!ptr || ptr->leafBlock) {
            std::cerr << "[moveFlipCut] Found invalid internal or leafBlock on internal.\n";
            std::abort();
        }
    }

    std::uniform_int_distribution<int> dist(0, int(_allInternals.size() - 1));
    int idx = dist(_rng);
    TreeNode* inode = _allInternals[idx];
    assert(inode != nullptr && inode->leafBlock == nullptr);

    inode->savedCutType     = inode->cutIsHorizontal;
    inode->cutIsHorizontal  = !inode->cutIsHorizontal;
    return true;
}

/************************************************************************
 *                     Annealing Acceptance Check                      *
 ************************************************************************/
/// Dual-temperature Metropolis:
///   Accept iff (Δarea ≤ 0 OR exp(−Δarea / T_area) > U[0,1))
///         AND (Δwire ≤ 0 OR exp(−Δwire / T_wire) > U[0,1))
bool MacroPlacer::acceptNeighbor(double deltaArea, double deltaWire)
{
    bool acceptA = (deltaArea <= 0.0) ||
                   (_uniform01(_rng) < std::exp(-deltaArea / std::max(_T_area,1e-9)));
    bool acceptW = (deltaWire <= 0.0) ||
                   (_uniform01(_rng) < std::exp(-deltaWire / std::max(_T_wire,1e-9)));
    return (acceptA && acceptW);
}

/************************************************************************
 *                          Annealing Loop                              *
 ************************************************************************/
void MacroPlacer::annealLoop()
{
    int outerIter = 0;
    while ((_T_area > _minTemp || _T_wire > _minTemp) && outerIter < 1000) {
        outerIter++;
        for (int i = 0; i < _innerIters; i++) {
            // 1) Measure current costs
            double oldArea = computeAreaCost();
            double oldWire = computeWireCost();

            // 2) Backup old root so we can restore on reject
            TreeNode* oldTree = cloneTree(_root);
            std::vector<bool> oldRotated(_blocks.size());
            for (auto& b : _blocks) oldRotated[b.id] = b.rotated;

            // 3) Make one random move (wrapped in try/catch & reindex)
            bool moved = applyRandomMove();
            if (!moved) {
                // we already rolled back inside applyRandomMove()
                destroyTree(oldTree);
                continue;
            }

            // 4) Evaluate new costs
            double newArea = computeAreaCost();
            double newWire = computeWireCost();
            double dA = newArea - oldArea;
            double dW = newWire - oldWire;

            // 5) Accept or reject
            if (acceptNeighbor(dA, dW)) {
                // If better, update “best”
                if (newArea + newWire < _bestCostArea + _bestCostWire) {
                    _bestCostArea = newArea;
                    _bestCostWire = newWire;
                    destroyTree(_bestRoot);
                    _bestRoot = cloneTree(_root);
                }
                destroyTree(oldTree);
            }
            else {
                // Reject: restore old tree & orientations
                destroyTree(_root);
                _root = oldTree;
                for (auto &b : _blocks) b.rotated = oldRotated[b.id];

                // Reindex after rollback
                _allInternals.clear();
                _allLeaves.clear();
                indexTreeNodes(_root);

                if (_allLeaves.empty() || _allInternals.empty()) {
                    std::cerr << "[ERROR] Tree invalid after rollback.\n";
                    debugPrintTree(_root, 0);
                    std::abort();
                }
            }
        }

        // 6) Cool temperatures
        _T_area *= _alphaA;
        _T_wire *= _alphaW;

        if (outerIter % 10 == 0) {
            std::cout << "[Anneal] Iter " << outerIter
                      << "  T_area=" << _T_area
                      << "  T_wire=" << _T_wire
                      << "  BestArea=" << _bestCostArea
                      << "  BestWire=" << _bestCostWire << "\n";
        }
    }

    // Restore best solution
    destroyTree(_root);
    _root = cloneTree(_bestRoot);
    computeAreaCost();  // pack final leaf (x,y)
}

void MacroPlacer::runAnnealing()
{
    annealLoop();
    std::cout << "[MacroPlacer] Done annealing. Final AreaCost="
              << _bestCostArea << "  WireCost=" << _bestCostWire << "\n";
}

/************************************************************************
 *                    Tree Cloning / Saving / Restoring                 *
 ************************************************************************/
/// Deep‐copy the entire subtree rooted at `src`.
/// Copy leaf pointers but create new TreeNode objects.
TreeNode* MacroPlacer::cloneTree(TreeNode* src)
{
    if (!src) return nullptr;
    TreeNode* copy;
    if (src->leafBlock) {
        // Leaf: point to same Block, but new node
        copy = new TreeNode(src->leafBlock);
    } else {
        // Internal: clone children recursively
        TreeNode* lC = cloneTree(src->left);
        TreeNode* rC = cloneTree(src->right);
        copy = new TreeNode(lC, rC, src->cutIsHorizontal);
    }
    return copy;
}

/************************************************************************
 *                           Utility Routines                           *
 ************************************************************************/
void MacroPlacer::debugPrintTree(TreeNode* node, int indent)
{
    if (!node) return;
    for (int i = 0; i < indent; i++) std::cout << "  ";
    if (node->leafBlock) {
        std::cout << "Leaf(Block " << node->leafBlock->id
                  << ", rot=" << node->leafBlock->rotated << ")\n";
    } else {
        std::cout << "Internal(cut="
                  << (node->cutIsHorizontal ? "H" : "V") << ")\n";
        debugPrintTree(node->left, indent+1);
        debugPrintTree(node->right, indent+1);
    }
}

int MacroPlacer::countLeaves(TreeNode* node)
{
    if (!node) return 0;
    if (node->leafBlock) return 1;
    return countLeaves(node->left) + countLeaves(node->right);
}

/************************************************************************
 *          applyRandomMove() with try/catch, rollback, and reindex      *
 ************************************************************************/
bool MacroPlacer::applyRandomMove()
{
    // 1) Backup the current tree and leaf rotations
    TreeNode* backupRoot = cloneTree(_root);
    std::vector<bool> backupRotated(_blocks.size());
    for (auto &b : _blocks) {
        backupRotated[b.id] = b.rotated;
    }

    bool moveSucceeded = false;
    try {
        // 2) Compute “big‐move” probability
        double tempAvg = 0.5 * (_T_area + _T_wire);
        double pBigMove = std::clamp(
            tempAvg / (_bestCostArea + _bestCostWire + 1e-9),
            0.0,
            1.0
        );

        // 3) Choose and apply one of the four moves
        if (_uniform01(_rng) < pBigMove) {
            // Big move: swap or cut‐and‐reinsert
            if (_uniform01(_rng) < 0.5) {
                moveSucceeded = moveSwapTwoLeaves();
            } else {
                moveSucceeded = moveCutAndReinsertLeaf();
            }
        } else {
            // Small move: rotate or flip
            if (_uniform01(_rng) < 0.5) {
                moveSucceeded = moveRotateLeaf();
            } else {
                moveSucceeded = moveFlipCut();
            }
        }

        // If the primitive move returned false, throw an exception
        if (!moveSucceeded) {
            throw std::runtime_error("Primitive move returned false");
        }

        // 4) Now that structure changed, reindex both arrays
        _allInternals.clear();
        _allLeaves.clear();
        indexTreeNodes(_root);

        if (_allLeaves.empty() || _allInternals.empty()) {
            throw std::runtime_error("Tree invalid after reindex");
        }
    }
    catch (...) {
        // 5) On any error/exception, restore the previous tree + rotations
        destroyTree(_root);
        _root = backupRoot;
        for (auto &b : _blocks) {
            b.rotated = backupRotated[b.id];
        }

        // Reindex after rollback
        _allInternals.clear();
        _allLeaves.clear();
        indexTreeNodes(_root);

        if (_allLeaves.empty() || _allInternals.empty()) {
            std::cerr << "[ERROR] Tree invalid even after rollback.\n";
            debugPrintTree(_root, 0);
            std::abort();
        }

        return false;
    }

    // If we reached here, the move succeeded and tree is consistent
    return true;
}
