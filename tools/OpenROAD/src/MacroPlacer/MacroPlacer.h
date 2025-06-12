// ----------------------------------------------
// File: MacroPlacer.h
// ----------------------------------------------
#pragma once

#include <vector>
#include <utility>
#include <random>

/// A “Block” represents one macro cell.  It has an ID, an (x,y) placement,
/// a rotated flag, and fixed original width/height (from LEF).
struct Block
{
  int    id;           ///< Unique integer ID (0..N-1)
  double x, y;         ///< Lower-left coordinate (updated by the placer)
  bool   rotated;      ///< true if we have swapped width/height

  double width_orig;   ///< “LEF” width (unrotated)
  double height_orig;  ///< “LEF” height (unrotated)

  Block() : id(-1), x(0), y(0), rotated(false), width_orig(0), height_orig(0) {}
  Block(int _id, double w, double h)
    : id(_id), x(0), y(0), rotated(false), width_orig(w), height_orig(h) {}

  /// Return width after rotation
  double placedWidth() const
  {
    return (rotated ? height_orig : width_orig);
  }
  /// Return height after rotation
  double placedHeight() const
  {
    return (rotated ? width_orig : height_orig);
  }

  /// Center X coordinate
  double cx() const { return x + placedWidth()  * 0.5; }
  /// Center Y coordinate
  double cy() const { return y + placedHeight() * 0.5; }
};

/// A “Net” connects multiple macros (by their integer IDs).  We compute
/// HPWL across all blockIDs[].  The weight field can be used if some nets
/// are “heavier” in the cost function.
struct Net
{
  std::vector<int> blockIDs;  ///< list of connected Block::id’s
  double           weight;    ///< net weight in wire-length cost

  Net() : weight(1.0) {}
};

/// Each TreeNode is either:
///  - a leaf (leafBlock != nullptr), pointing to exactly one Block
///  - an internal node (leafBlock == nullptr), with two children, plus a
///    cutIsHorizontal flag to indicate H-cut or V-cut.
struct TreeNode
{
  Block*           leafBlock;       ///< non-null if this node is a leaf
  TreeNode*        left;            ///< left child (or bottom child if H-cut)
  TreeNode*        right;           ///< right child (or top child if H-cut)
  bool             cutIsHorizontal; ///< true = children stacked vertically (H cut)

  // These get assigned during packing (dfsPack)
  double width;   ///< subtree width (computed)
  double height;  ///< subtree height (computed)

  // For undo/rollback: store prior children pointers and cutType
  std::pair<TreeNode*,TreeNode*> savedChildren;
  bool                           savedCutType;

  /// Construct a leaf node
  TreeNode(Block* b)
    : leafBlock(b),
      left(nullptr),
      right(nullptr),
      cutIsHorizontal(false),
      width(0), height(0),
      savedChildren({nullptr,nullptr}),
      savedCutType(false)
  {}

  /// Construct an internal node whose two children are A and B, with cutType
  TreeNode(TreeNode* A, TreeNode* B, bool cutH)
    : leafBlock(nullptr),
      left(A),
      right(B),
      cutIsHorizontal(cutH),
      width(0), height(0),
      savedChildren({nullptr,nullptr}),
      savedCutType(cutH)
  {}
};

/// The main MacroPlacer class.  You pass in a list of macros (blocks) and a
/// list of nets.  It builds a random initial slicing tree, then runs a
/// dual-temperature simulated annealing to minimize (area + HPWL).
class MacroPlacer
{
public:
  /// Constructor: takes a vector of Blocks and a vector of Nets.
  MacroPlacer(std::vector<Block>& blocks,
              const std::vector<Net>& nets);

  /// Destructor: cleans up the primary tree and the “best” tree
  ~MacroPlacer()
  {
    destroyTree(_root);
    destroyTree(_bestRoot);
  }

  /// Run the full annealing-based placement
  void runAnnealing();

private:
  std::vector<Block>& _blocks;   ///< reference to user’s Block array
  const std::vector<Net>& _nets; ///< reference to user’s Net array

  TreeNode* _root        = nullptr; ///< current slicing-tree root
  TreeNode* _bestRoot    = nullptr; ///< best slicing-tree found so far

  std::vector<TreeNode*> _allLeaves;    ///< pointers to all leaves in current tree
  std::vector<TreeNode*> _allInternals; ///< pointers to all internal nodes

  std::mt19937                    _rng;       ///< random-number generator
  std::uniform_real_distribution<> _uniform01; ///< uniform [0,1) RNG

  // Annealing parameters:
  double _T_area      = 0.0;   ///< current “temperature” for area
  double _T_wire      = 0.0;   ///< current “temperature” for wire
  double _alphaA      = 0.0;   ///< cooling rate for area
  double _alphaW      = 0.0;   ///< cooling rate for wire
  int    _innerIters  = 0;     ///< number of trial moves per temperature
  double _minTemp     = 0.0;   ///< stop when both temps drop below

  // Cost of the best solution seen so far
  double _bestCostArea = 0.0;
  double _bestCostWire = 0.0;

  // ────────────────────────────────────────────────────────────────────────
  //               Initialization / Cleanup / Indexing Methods
  // ────────────────────────────────────────────────────────────────────────

  /// Recursively delete the subtree rooted at `n`
  void destroyTree(TreeNode* n);

  /// Build a random initial slicing tree linking all _allLeaves
  void buildInitialSlicingTree();

  /// Walk the current tree rooted at `node` and append every leaf to _allLeaves
  /// and every internal node to _allInternals.
  void indexTreeNodes(TreeNode* node);

  // ────────────────────────────────────────────────────────────────────────
  //                       Cost / Packing Methods
  // ────────────────────────────────────────────────────────────────────────

  /// Recursively pack the subtree at `node` into (originX,originY).  Returns (w,h).
  std::pair<double,double> dfsPack(TreeNode* node,
                                   double originX,
                                   double originY);

  /// Pack the entire tree at (0,0) and return total area (W * H).
  double packTree(TreeNode* node, double originX, double originY);

  /// Compute area cost = bounding-box area of the root packing
  double computeAreaCost();

  /// Compute total HPWL over all nets, given current leaf placements
  double computeWireCost();

  // ────────────────────────────────────────────────────────────────────────
  //                         Primitive Move Operations
  // ────────────────────────────────────────────────────────────────────────

  /// (1) Swap the leafBlock pointers of two randomly chosen leaves
  bool moveSwapTwoLeaves();

  /// (2) Cut one leaf out of the tree and re-insert it under a random internal
  bool moveCutAndReinsertLeaf();

  /// (3) Flip the rotated flag of one randomly chosen leaf
  bool moveRotateLeaf();

  /// (4) Flip (H ↔ V) on one randomly chosen internal node
  bool moveFlipCut();

  // ────────────────────────────────────────────────────────────────────────
  //                          Annealing / Acceptance
  // ────────────────────────────────────────────────────────────────────────

  /// Perform one random move (swap, re-insert, rotate, or flip)
  /// wrapped in try/catch to rollback on error, and reindex after success
  bool applyRandomMove();

  /// Dual-temperature Metropolis acceptance
  bool acceptNeighbor(double deltaArea, double deltaWire);

  void annealLoop();

  // ────────────────────────────────────────────────────────────────────────
  //                     Tree-Clone / Backup / Restore
  // ────────────────────────────────────────────────────────────────────────

  /// Deep-copy the subtree rooted at `src`, returning a brand-new tree
  TreeNode* cloneTree(TreeNode* src);

public:
  /// Public entry: run the annealing until convergence, then fix final block (x,y)
  void run() { runAnnealing(); }

  // ────────────────────────────────────────────────────────────────────────
  //                            Debug / Extras
  // ────────────────────────────────────────────────────────────────────────

  /// Print the tree structure (for debugging)
  void debugPrintTree(TreeNode* node, int indent = 0);

  /// Count the number of leaves under `node`
  int countLeaves(TreeNode* node);
};
