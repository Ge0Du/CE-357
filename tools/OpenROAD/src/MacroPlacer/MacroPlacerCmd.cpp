// ----------------------------------------------
// File: MacroPlaceCmd.cpp
// ----------------------------------------------
// This file registers a Tcl command named “macro_place” that runs the placer.

#include "MacroPlacer.h"
#include <odb/db.h>            // for odb::dbDatabase, odb::dbBlock, dbInst, etc.
#include <ord/OpenRoad.hh>     // for ord::OpenRoad::openRoad()->getDb()
#include <tcl.h>               // for Tcl_CreateCommand, Tcl_SetResult, etc.
#include <cmath>               // for std::round
#include <iostream>
#include <unordered_map>
#include <vector>

namespace macplacer {

static int macroPlaceCustom(ClientData, Tcl_Interp* tcl_interp, int argc, const char** argv)
{
    if (argc != 1) {
        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Usage: macro_place"),
                      TCL_STATIC);
        return TCL_ERROR;
    }

    // 1) Grab the global ODB pointer from OpenROAD
    ord::OpenRoad* openroad = ord::OpenRoad::openRoad();
    if (!openroad) {
        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Error: OpenRoad instance not found."),
                      TCL_STATIC);
        return TCL_ERROR;
    }
    odb::dbDatabase* db = openroad->getDb();
    if (!db) {
        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Error: No dbDatabase available."),
                      TCL_STATIC);
        return TCL_ERROR;
    }
    odb::dbBlock* block = db->getChip() ? db->getChip()->getBlock() : nullptr;
    if (!block) {
        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Error: No top block found in ODB."),
                      TCL_STATIC);
        return TCL_ERROR;
    }

    // 2) Build vectors of Blocks and Nets from the ODB design
    std::vector<Block>             blocks;
    std::vector<Net>               nets;
    std::unordered_map<odb::dbInst*,int> inst2id;
    std::vector<odb::dbInst*>      id2inst;
    int                             nextId = 0;

    // Collect all macros (dbInst whose master->isBlock())
    for (auto inst : block->getInsts()) {
        if (inst->getMaster()->isBlock()) {
            std::cout << "  selecting inst " << inst->getName()
                      << "  master=" << inst->getMaster()->getName() << "\n";

            odb::dbBox* bbox = inst->getBBox();
            if (!bbox) {
                Tcl_SetResult(tcl_interp,
                              const_cast<char*>("Error: Found macro with no BBox."),
                              TCL_STATIC);
                return TCL_ERROR;
            }
            double w_dbu = bbox->getDX();
            double h_dbu = bbox->getDY();
            if (w_dbu <= 0 || h_dbu <= 0) {
                Tcl_SetResult(tcl_interp,
                              const_cast<char*>("Error: Macro has non-positive dimension."),
                              TCL_STATIC);
                return TCL_ERROR;
            }

            blocks.emplace_back(nextId, w_dbu, h_dbu);
            inst2id[inst] = nextId;
            id2inst.push_back(inst);
            nextId++;
        }
    }
    if (blocks.size() < 2) {
        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Error: Need at least two macros to place."),
                      TCL_STATIC);
        return TCL_ERROR;
    }

    // Build nets: each net that connects ≥2 macros → Net{ids[], weight=1.0}
    for (auto net : block->getNets()) {
        std::vector<int> endpoints;
        for (auto iterm : net->getITerms()) {
            auto owner = iterm->getInst();
            auto it    = inst2id.find(owner);
            if (it != inst2id.end()) {
                endpoints.push_back(it->second);
            }
        }
        if (endpoints.size() >= 2) {
            Net n;
            n.blockIDs = std::move(endpoints);
            n.weight   = 1.0;
            nets.push_back(n);
        }
    }
    if (nets.empty()) {
        Tcl_SetResult(tcl_interp,
                      const_cast<char*>("Error: No nets connecting two or more macros."),
                      TCL_STATIC);
        return TCL_ERROR;
    }

    // 3) Instantiate and run the placer
    MacroPlacer placer(blocks, nets);

    // (Optional) Debug: print each block’s “before” coordinate (they default to 0,0)
    for (auto &b : blocks) {
        std::cout << "  pre-place  Block " << b.id
                  << " initially at (" << b.x << "," << b.y << ")\n";
    }

    placer.run();  // runs annealing, updates each Block’s (x,y,rotated)

    // (Optional) Debug: print each block’s “after” coordinate
    for (auto &b : blocks) {
        std::cout << "  post-place Block " << b.id
                  << " ended at (" << b.x << "," << b.y << ")\n";
    }

    // 4) Write back each Block’s (x,y,rotated) into the corresponding dbInst
    //    → Also mark each inst as “FIRM” to force ODB to recognize it as placed.
    //
    for (auto &b : blocks) {
        odb::dbInst* inst = id2inst[b.id];
        if (!inst) {
            std::cerr << "[Error] Unable to find dbInst for Block ID "
                      << b.id << "\n";
            continue;
        }

        // Round to the nearest integer DBU
        int x_dbu = static_cast<int>(std::round(b.x));
        int y_dbu = static_cast<int>(std::round(b.y));

        inst->setLocation(x_dbu, y_dbu);


        if (b.rotated) {
            inst->setOrient(odb::dbOrientType::R90);
        } else {
            inst->setOrient(odb::dbOrientType::R0);
        }
        // ←――――――――――――――――――――――――――――――――――――――――――――
        // **This line is crucial**.  If you omit it, OpenROAD will still
        // think the instance is “unplaced,” so later stages (TAP, PDN, GRT)
        // cannot see its pins.  
        inst->setPlacementStatus( odb::dbPlacementStatus::FIRM );
        // ←――――――――――――――――――――――――――――――――――――――――――――

    }

    std::cout << "[macro_place] Macro placement complete.\n";
    return TCL_OK;
}

/// Factory function that OpenROAD calls on startup to register “macro_place.”
extern "C" int ord__init_openroad_macroplacer(Tcl_Interp* interp)
{
    Tcl_CreateCommand(interp,
                      "macro_place",           // Tcl command name
                      (Tcl_CmdProc*)macroPlaceCustom,
                      (ClientData)NULL,
                      (Tcl_CmdDeleteProc*)NULL);

    Tcl_SetResult(interp,
                  const_cast<char*>("Loaded macro_placer"),
                  TCL_STATIC);
    return TCL_OK;
    }

}  // namespace macplacer
