#include <tcl.h>
#include <iostream>
#include "MacroPlacer.h"
#include "odb/db.h"

// Forward declaration to get dbBlock

namespace ord {
  odb::dbDatabase* getDb();

  int macro_place_tcl(ClientData, Tcl_Interp* interp, int argc, const char* argv[])
  {
    odb::dbDatabase* db = getDb();
    odb::dbBlock* block = db->getChip()->getBlock();
    if (!block) {
      std::cerr << "Error: No design loaded.\n";
      return TCL_ERROR;
    }

    macroplacer::MacroPlacer placer(block);
    placer.run();
    std::cout << "Macro placement complete." << std::endl;
    return TCL_OK;
  }
}

// 👇 extern "C" outside namespace!
extern "C" void ord__init_openroad_macroplacer(Tcl_Interp* interp)
{
  Tcl_CreateCommand(interp, "macro_place", ord::macro_place_tcl, nullptr, nullptr);
}
