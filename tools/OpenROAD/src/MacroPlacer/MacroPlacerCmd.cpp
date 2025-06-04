#include "MacroPlacer.h"
#include "tcl/Command.hh"
#include "odb/db.h"

namespace macroplacer {

// Forward declaration of OpenROAD database access
namespace ord {
extern odb::dbDatabase* getDb();
}

class MacroPlaceCmd : public ord::Command {
public:
  MacroPlaceCmd() : ord::Command("macro_place") {}

  int execute() override {
    odb::dbDatabase* db = ord::getDb();
    odb::dbBlock* block = db->getChip()->getBlock();
    if (!block) {
      std::cerr << "Error: No design loaded.\n";
      return 1;
    }

    MacroPlacer placer(block);
    placer.run();

    std::cout << "Macro placement complete.\n";
    return 0;
  }
};

ord::Command* createMacroPlaceCmd() {
  return new MacroPlaceCmd();
}

}  // namespace macroplacer
