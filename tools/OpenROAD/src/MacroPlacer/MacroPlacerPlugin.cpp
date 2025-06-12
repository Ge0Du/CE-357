// ============================================================================
// File: src/custom/macro_placer_plugin.cpp
// Purpose: “Bridge” between OpenROAD’s ODB and our standalone MacroPlacer
// ============================================================================


#include "MacroPlacer.h"          // our custom Dual‐Temperature MacroPlacer
#include <odb/db.h>               // OpenDB’s primary header (db::dbBlock, dbInst, etc.)
#include <ord/OpenRoad.hh>   // OpenROAD API (ord::OpenRoad::openRoad(), etc.)
#include <sta/Sta.hh>             // (optional—for timing, not strictly needed here)
 #include <tcl.h>
 #include <set>
 #include <unordered_map>

using namespace odb;
using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utility: Convert ODB orientation enum to a bool “rotated” and back
////////////////////////////////////////////////////////////////////////////////////////////////////////

/// We’ll treat “rotated = true” as meaning 90° (south‐east origin) vs. “rotated = false” as 0° (north‐east).
static bool measureRotated(dbOrientType orient)
{
    // ODB’s orient enum:  (see odb::dbOrientType in OpenDB docs)
    //   R0   = no rotation
    //   R90  = rotated 90° clockwise
    //   R180 = 180°
    //   R270 = 270°
    //   MX   = mirrored along X
    //   MY   = mirrored along Y
    //   MXR90= mirrored+rotated
    //   MYR90= mirrored+rotated
    //
    // We’ll treat any R90‐based orientation as “rotated = true.”  Everything else → false.
    return (orient == dbOrientType::R90 || orient == dbOrientType::MYR90
         || orient == dbOrientType::MXR90 || orient == dbOrientType::R270);
}

/// Map our “rotated” bool back to an ODB orientation (we choose R0 or R90)
static dbOrientType toDbOrient(bool rotated)
{
    return rotated ? dbOrientType::R90 : dbOrientType::R0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Step 1: We need to gather all MACROS in the top‐level block and build (Block, Net) vectors
////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Helper: Return true if `inst` is a “macro instance” (i.e. a hard macro or a virtual macro).
static bool isMacroInst(dbInst* inst)
{
    // If the master is a block, it’s a full‐custom macro.  If it’s a leaf cell (std cell), skip.
    // (You can add further filters if you have “virtual” macros in LEF that you want to skip.)
    return inst->getMaster()->isBlock();
}

/// We will assign each macro a consecutive integer ID [0..N−1], in the order we find them.
/// We also store a map from macro‐ID → dbInst* so we can write results back.
struct MacroInfo {
    int        id;
    dbInst*    instPtr;
    Block      b;       // Our “Block” struct (id, w, h, x, y, rotated)
};

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Step 2: Build a netlist that only includes nets connecting 2+ of our macros
////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Helper: Given a dbNet*, find which of our macro IDs connect to it.  Return a vector<int> of macro‐IDs.
static vector<int> collectMacroEndpoints(dbNet* net, const unordered_map<dbInst*, int>& inst2id)
{
    vector<int> endpoints;
    for (auto iterm : net->getITerms()) {
      dbInst* inst = iterm->getInst();
      auto it = inst2id.find(inst);
      if (it != inst2id.end()) {
        endpoints.push_back(it->second);
      }
    }

    // We only care about nets that actually connect 2 or more macros.
    if (endpoints.size() >= 2) {
        // Remove duplicates, just in case a macro has multiple pins on the same net
        sort(endpoints.begin(), endpoints.end());
        endpoints.erase(unique(endpoints.begin(), endpoints.end()), endpoints.end());
        if (endpoints.size() >= 2)
            return endpoints;
    }
    return {};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Step 3: The TCL command itself: “macroPlaceCustom”
////////////////////////////////////////////////////////////////////////////////////////////////////////

static int macroPlaceCustom(ClientData clientData,
                            Tcl_Interp* tcl_interp,
                            int argc,
                            const char** argv){
    // We expect no arguments: just “macroPlaceCustom”
    if (argc != 1) {
        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Usage: macroPlaceCustom"),
                      TCL_STATIC);
        return TCL_ERROR;
    }

    // 1) Grab the top-level design from OpenROAD
    ord::OpenRoad* openroad = ord::OpenRoad::openRoad();
    if (!openroad) {
        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Error: OpenRoad instance not found."),
                      TCL_STATIC);
        return TCL_ERROR;
    }

    // We assume the user has already read in all LEF/DEF and written the design to ODB.
    dbDatabase* db = openroad->getDb();
    dbBlock*   block = db->getChip()->getBlock();
    if (!block) {
        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Error: No top-level block found in ODB."),
                      TCL_STATIC);
        return TCL_ERROR;
    }

    // 2) Collect all macros in the block
    vector<MacroInfo> macros;
    macros.reserve(block->getInsts().size());
    unordered_map<dbInst*, int> inst2id; // maps each dbInst* → macro ID
    int nextId = 0;

    for (auto inst : block->getInsts()) {
        if (isMacroInst(inst)) {
            // get the geometry (DX, DY) from the geometry of the master’s bbox
            // Note: For hard macros, LEF/DEF sets the bounding box so getBBox() is valid.
            dbBox* bbox = inst->getBBox();
            double w = bbox->getDX();
            double h = bbox->getDY();

            MacroInfo mi;
            mi.id = nextId;
            mi.instPtr = inst;
            mi.b = Block(nextId, w, h);
            macros.push_back(mi);
            inst2id[inst] = nextId;
            nextId++;
        }
    }

    if (macros.size() < 2) {
        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Error: Need at least two macros to run custom placer."),
                      TCL_STATIC);
        return TCL_ERROR;
    } {
        std::string msg = "puts \"[macroPlaceCustom] Found "
                          + std::to_string(macros.size())
                          + " macros.\"";
        Tcl_Eval(tcl_interp, msg.c_str());
    }

    // 3) Build the netlist of only those nets that connect two or more of our macros
    vector<Net> netlist;
    netlist.reserve(block->getNets().size());

    for (auto net : block->getNets()) {
        vector<int> ends = collectMacroEndpoints(net, inst2id);
        if (!ends.empty()) {
            Net n;
            n.blockIDs = std::move(ends);
            n.weight = 1.0; // You can adjust per-net weight if desired
            netlist.push_back(n);
        }
    }
{
        std::string msg = "puts \"[macroPlaceCustom] Built netlist with "
                          + std::to_string(netlist.size())
                          + " nets.\"";
        Tcl_Eval(tcl_interp, msg.c_str());
    }

    // 4) Prepare the vector<Block> for MacroPlacer
    //    We need to pass a contiguous std::vector<Block> (not MacroInfo).
    vector<Block> blockVector;
    blockVector.reserve(macros.size());
    for (auto &mi : macros) {
        blockVector.push_back(mi.b);
    }

    // 5) Run our MacroPlacer
    MacroPlacer placer(blockVector, netlist);
    placer.runAnnealing();

    // 6) Read back final (x,y,rotated) from placer.getBlocks() and write into ODB
    const vector<Block>& finalBlocks = placer.getBlocks();
    for (auto &b : finalBlocks) {
        int id = b.id;
        double x = b.x;
        double y = b.y;
        bool  rot = b.rotated;

        // Compare with the MacroInfo vector to find the dbInst*
        dbInst* inst = macros[id].instPtr;
        if (!inst) continue;

        // Set the location & orientation on that instance
        inst->setLocation(x, y);
        inst->setOrient(toDbOrient(rot));
    }

    Tcl_Eval(tcl_interp, "puts \"[macroPlaceCustom] Placed macros in ODB successfully.\"");
    return TCL_OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Step 4: Register the TCL command when OpenROAD starts up
////////////////////////////////////////////////////////////////////////////////////////////////////////

extern "C" {
    int ord__init_openroad_macroplacer(Tcl_Interp* tcl_interp)    {
        // Create the “macroPlaceCustom” command
        Tcl_CreateCommand(tcl_interp,
                          "MacroPlacer",
                          (Tcl_CmdProc*)MacroPlacer,
                          (ClientData)NULL,
                          (Tcl_CmdDeleteProc*)NULL);

        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Loaded macro_placer_plugin"),
                      TCL_STATIC);
        return TCL_OK;
    }
}

