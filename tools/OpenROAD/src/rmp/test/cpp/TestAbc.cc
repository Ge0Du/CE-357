// Copyright 2023 Google LLC
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include <tcl.h>
#include <unistd.h>

#include <array>
#include <cstddef>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "abc_library_factory.h"
#include "base/abc/abc.h"
#include "base/io/ioAbc.h"
#include "base/main/abcapis.h"
#include "db_sta/MakeDbSta.hh"
#include "db_sta/dbReadVerilog.hh"
#include "db_sta/dbSta.hh"
#include "delay_optimization_strategy.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "logic_extractor.h"
#include "map/mio/mio.h"
#include "map/scl/sclLib.h"
#include "odb/lefin.h"
#include "sta/FuncExpr.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/NetworkClass.hh"
#include "sta/PortDirection.hh"
#include "sta/Sta.hh"
#include "sta/Units.hh"
#include "sta/VerilogReader.hh"
#include "sta/VerilogWriter.hh"
#include "utl/Logger.h"
#include "utl/deleter.h"
#include "zero_slack_strategy.h"

// Headers have duplicate declarations so we include
// a forward one to get at this function without angering
// gcc.
namespace abc {
void* Abc_FrameReadLibGen();
}

namespace rmp {

using ::testing::Contains;

std::once_flag init_sta_flag;

class AbcTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    db_ = utl::UniquePtrWithDeleter<odb::dbDatabase>(odb::dbDatabase::create(),
                                                     &odb::dbDatabase::destroy);
    std::call_once(init_sta_flag, []() {
      sta::initSta();
      abc::Abc_Start();
    });
    db_->setLogger(&logger_);
    sta_ = std::unique_ptr<sta::dbSta>(sta::makeDbSta());
    sta_->initVars(Tcl_CreateInterp(), db_.get(), &logger_);
    auto path = std::filesystem::canonical("./Nangate45/Nangate45_typ.lib");
    library_ = sta_->readLiberty(path.string().c_str(),
                                 sta_->findCorner("default"),
                                 /*min_max=*/sta::MinMaxAll::all(),
                                 /*infer_latches=*/false);

    odb::lefin lef_reader(
        db_.get(), &logger_, /*ignore_non_routing_layers=*/false);

    auto tech_lef
        = std::filesystem::canonical("./Nangate45/Nangate45_tech.lef");
    auto stdcell_lef
        = std::filesystem::canonical("./Nangate45/Nangate45_stdcell.lef");
    odb::dbTech* tech
        = lef_reader.createTech("nangate45", tech_lef.string().c_str());
    odb::dbLib* lib
        = lef_reader.createLib(tech, "nangate45", stdcell_lef.string().c_str());

    sta_->postReadLef(/*tech=*/nullptr, lib);

    sta::Units* units = library_->units();
    power_unit_ = units->powerUnit();
  }

  void LoadVerilog(const std::string& file_name, const std::string& top = "top")
  {
    // Assumes module name is "top" and clock name is "clk"
    sta::dbNetwork* network = sta_->getDbNetwork();
    ord::dbVerilogNetwork verilog_network;
    verilog_network.init(network);

    sta::VerilogReader verilog_reader(&verilog_network);
    verilog_reader.read(file_name.c_str());

    ord::dbLinkDesign(top.c_str(),
                      &verilog_network,
                      db_.get(),
                      &logger_,
                      /*hierarchy = */ false);

    sta_->postReadDb(db_.get());

    sta::Cell* top_cell = network->cell(network->topInstance());
    sta::Port* clk_port = network->findPort(top_cell, "clk");
    sta::Pin* clk_pin = network->findPin(network->topInstance(), clk_port);

    sta::PinSet* pinset = new sta::PinSet(network);
    pinset->insert(clk_pin);

    // 0.5ns
    double period = sta_->units()->timeUnit()->userToSta(0.5);
    sta::FloatSeq* waveform = new sta::FloatSeq;
    waveform->push_back(0);
    waveform->push_back(period / 2.0);

    sta_->makeClock("core_clock",
                    pinset,
                    /*add_to_pins=*/false,
                    /*period=*/period,
                    waveform,
                    /*comment=*/nullptr);

    sta_->ensureGraph();
    sta_->ensureLevelized();
  }
  std::map<std::string, int> AbcLogicNetworkNameToPrimaryOutputIds(
      abc::Abc_Ntk_t* network)
  {
    std::map<std::string, int> primary_output_name_to_index;
    for (int i = 0; i < abc::Abc_NtkPoNum(network); i++) {
      abc::Abc_Obj_t* po = abc::Abc_NtkPo(network, i);
      std::string po_name = abc::Abc_ObjName(po);
      primary_output_name_to_index[po_name] = i;
    }

    return primary_output_name_to_index;
  }

  utl::UniquePtrWithDeleter<odb::dbDatabase> db_;
  sta::Unit* power_unit_;
  std::unique_ptr<sta::dbSta> sta_;
  sta::LibertyLibrary* library_;
  utl::Logger logger_;
};

class AbcTestSky130 : public AbcTest
{
  void SetUp() override
  {
    db_ = utl::UniquePtrWithDeleter<odb::dbDatabase>(odb::dbDatabase::create(),
                                                     &odb::dbDatabase::destroy);
    std::call_once(init_sta_flag, []() {
      sta::initSta();
      abc::Abc_Start();
    });
    db_->setLogger(&logger_);
    sta_ = std::unique_ptr<sta::dbSta>(sta::makeDbSta());
    sta_->initVars(Tcl_CreateInterp(), db_.get(), &logger_);
    auto path = std::filesystem::canonical(
        "./sky130/sky130_fd_sc_hd__ss_n40C_1v40.lib");
    library_ = sta_->readLiberty(path.string().c_str(),
                                 sta_->findCorner("default"),
                                 /*min_max=*/sta::MinMaxAll::all(),
                                 /*infer_latches=*/false);

    odb::lefin lef_reader(
        db_.get(), &logger_, /*ignore_non_routing_layers=*/false);

    auto tech_lef = std::filesystem::canonical("./sky130/sky130hd.tlef");
    auto stdcell_lef
        = std::filesystem::canonical("./sky130/sky130hd_std_cell.lef");
    odb::dbTech* tech
        = lef_reader.createTech("sky130", tech_lef.string().c_str());
    odb::dbLib* lib
        = lef_reader.createLib(tech, "sky130", stdcell_lef.string().c_str());

    sta_->postReadLef(/*tech=*/nullptr, lib);

    sta::Units* units = library_->units();
    power_unit_ = units->powerUnit();
  }
};

class AbcTestAsap7 : public AbcTest
{
  void SetUp() override
  {
    db_ = utl::UniquePtrWithDeleter<odb::dbDatabase>(odb::dbDatabase::create(),
                                                     &odb::dbDatabase::destroy);
    std::call_once(init_sta_flag, []() {
      sta::initSta();
      abc::Abc_Start();
    });
    db_->setLogger(&logger_);
    sta_ = std::unique_ptr<sta::dbSta>(sta::makeDbSta());
    sta_->initVars(Tcl_CreateInterp(), db_.get(), &logger_);

    std::vector<std::string> liberty_paths
        = {"./asap7/asap7sc7p5t_AO_RVT_FF_nldm_211120.lib.gz",
           "./asap7/asap7sc7p5t_INVBUF_RVT_FF_nldm_220122.lib.gz",
           "./asap7/asap7sc7p5t_OA_RVT_FF_nldm_211120.lib.gz",
           "./asap7/asap7sc7p5t_SEQ_RVT_FF_nldm_220123.lib",
           "./asap7/asap7sc7p5t_SIMPLE_RVT_FF_nldm_211120.lib.gz"};

    for (const std::string& liberty_path : liberty_paths) {
      auto path = std::filesystem::canonical(liberty_path);
      library_ = sta_->readLiberty(path.string().c_str(),
                                   sta_->findCorner("default"),
                                   /*min_max=*/sta::MinMaxAll::all(),
                                   /*infer_latches=*/false);
    }

    odb::lefin lef_reader(
        db_.get(), &logger_, /*ignore_non_routing_layers=*/false);

    auto tech_lef
        = std::filesystem::canonical("./asap7/asap7_tech_1x_201209.lef");
    auto stdcell_lef
        = std::filesystem::canonical("./asap7/asap7sc7p5t_28_R_1x_220121a.lef");
    odb::dbTech* tech
        = lef_reader.createTech("asap7", tech_lef.string().c_str());
    odb::dbLib* lib
        = lef_reader.createLib(tech, "asap7", stdcell_lef.string().c_str());

    sta_->postReadLef(/*tech=*/nullptr, lib);

    sta::Units* units = library_->units();
    power_unit_ = units->powerUnit();
  }
};

TEST_F(AbcTest, CellPropertiesMatchOpenSta)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  for (size_t i = 0; i < Vec_PtrSize(&abc_library.abc_library()->vCells); i++) {
    abc::SC_Cell* abc_cell = static_cast<abc::SC_Cell*>(
        abc::Vec_PtrEntry(&abc_library.abc_library()->vCells, i));
    sta::LibertyCell* sta_cell = library_->findLibertyCell(abc_cell->pName);
    EXPECT_NE(nullptr, sta_cell);
    // Expect area matches
    EXPECT_FLOAT_EQ(abc_cell->area, sta_cell->area());

    float leakage_power = -1;
    bool exists;
    sta_cell->leakagePower(leakage_power, exists);
    if (exists) {
      EXPECT_FLOAT_EQ(abc_cell->leakage, power_unit_->staToUser(leakage_power));
    }
  }
}

TEST_F(AbcTest, DoesNotContainPhysicalCells)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  std::set<std::string> abc_cells;
  using ::testing::Contains;
  using ::testing::Not;

  for (size_t i = 0; i < Vec_PtrSize(&abc_library.abc_library()->vCells); i++) {
    abc::SC_Cell* abc_cell = static_cast<abc::SC_Cell*>(
        abc::Vec_PtrEntry(&abc_library.abc_library()->vCells, i));
    abc_cells.emplace(abc_cell->pName);
  }

  EXPECT_THAT(abc_cells, Not(Contains("ANTENNA_X1")));
  EXPECT_THAT(abc_cells, Not(Contains("FILLCELL_X1")));
}

TEST_F(AbcTestAsap7, ImportsWithoutErrors)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  EXPECT_NO_THROW(factory.Build());
}

TEST_F(AbcTest, DoesNotContainSequentialCells)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  std::set<std::string> abc_cells;
  using ::testing::Contains;
  using ::testing::Not;

  for (size_t i = 0; i < Vec_PtrSize(&abc_library.abc_library()->vCells); i++) {
    abc::SC_Cell* abc_cell = static_cast<abc::SC_Cell*>(
        abc::Vec_PtrEntry(&abc_library.abc_library()->vCells, i));
    abc_cells.emplace(abc_cell->pName);
  }

  EXPECT_THAT(abc_cells, Not(Contains("DFFRS_X2")));
}

TEST_F(AbcTest, ContainsLogicCells)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  std::set<std::string> abc_cells;
  using ::testing::Contains;
  using ::testing::Not;

  for (size_t i = 0; i < Vec_PtrSize(&abc_library.abc_library()->vCells); i++) {
    abc::SC_Cell* abc_cell = static_cast<abc::SC_Cell*>(
        abc::Vec_PtrEntry(&abc_library.abc_library()->vCells, i));
    abc_cells.emplace(abc_cell->pName);
  }

  EXPECT_THAT(abc_cells, Contains("AND2_X1"));
  EXPECT_THAT(abc_cells, Contains("AND2_X2"));
  EXPECT_THAT(abc_cells, Contains("AND2_X4"));
  EXPECT_THAT(abc_cells, Contains("AOI21_X1"));
}

// Create standard cell library from dbsta. Then create an
// abc network with a single and gate, and make sure that it
// simulates correctly.
TEST_F(AbcTest, TestLibraryInstallation)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  // When you set these params to zero they are essentially turned off.
  abc::Abc_SclInstallGenlib(abc_library.abc_library(),
                            /*Slew=*/0,
                            /*Gain=*/0,
                            /*fUseAll=*/0,
                            /*nGatesMin=*/0);
  abc::Mio_LibraryTransferCellIds();
  abc::Mio_Library_t* lib
      = static_cast<abc::Mio_Library_t*>(abc::Abc_FrameReadLibGen());

  std::map<std::string, abc::Mio_Gate_t*> gates;
  abc::Mio_Gate_t* gate = abc::Mio_LibraryReadGates(lib);
  while (gate) {
    gates[abc::Mio_GateReadName(gate)] = gate;
    gate = abc::Mio_GateReadNext(gate);
  }

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> network(
      abc::Abc_NtkAlloc(abc::Abc_NtkType_t::ABC_NTK_NETLIST,
                        abc::Abc_NtkFunc_t::ABC_FUNC_MAP,
                        /*fUseMemMan=*/1),
      &abc::Abc_NtkDelete);
  abc::Abc_NtkSetName(network.get(), strdup("test_module"));

  abc::Abc_Obj_t* input_1 = abc::Abc_NtkCreatePi(network.get());
  abc::Abc_Obj_t* input_1_net = abc::Abc_NtkCreateNet(network.get());
  abc::Abc_Obj_t* input_2 = abc::Abc_NtkCreatePi(network.get());
  abc::Abc_Obj_t* input_2_net = abc::Abc_NtkCreateNet(network.get());
  abc::Abc_Obj_t* output = abc::Abc_NtkCreatePo(network.get());
  abc::Abc_Obj_t* output_net = abc::Abc_NtkCreateNet(network.get());
  abc::Abc_Obj_t* and_gate = abc::Abc_NtkCreateNode(network.get());

  abc::Abc_ObjSetData(and_gate, gates["AND2_X1"]);

  abc::Abc_ObjAddFanin(input_1_net, input_1);
  abc::Abc_ObjAddFanin(input_2_net, input_2);

  // Gate order is technically dependent on the order in which the port
  // appears in the Mio_Gate_t struct. In practice you should go a build
  // a port_name -> index map type thing to make sure the right ports
  // are connected.
  abc::Abc_ObjAddFanin(and_gate, input_1_net);  // A
  abc::Abc_ObjAddFanin(and_gate, input_2_net);  // B

  std::string output_name = "out";
  abc::Abc_ObjAssignName(output_net, output_name.data(), /*pSuffix=*/nullptr);
  abc::Abc_ObjAddFanin(output_net, and_gate);

  abc::Abc_ObjAddFanin(output, output_net);

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> logic_network(
      abc::Abc_NtkToLogic(network.get()), &abc::Abc_NtkDelete);

  std::array<int, 2> input_vector = {1, 1};
  utl::UniquePtrWithDeleter<int> output_vector(
      abc::Abc_NtkVerifySimulatePattern(logic_network.get(),
                                        input_vector.data()),
      &free);

  EXPECT_EQ(output_vector.get()[0], 1);  // Expect that 1 & 1 == 1
}

TEST_F(AbcTest, ExtractsAndGateCorrectly)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  LoadVerilog("simple_and_gate_extract.v");

  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Vertex* flop_input_vertex = nullptr;
  for (sta::Vertex* vertex : *sta_->endpoints()) {
    if (std::string(vertex->name(network)) == "output_flop/D") {
      flop_input_vertex = vertex;
    }
  }
  EXPECT_NE(flop_input_vertex, nullptr);

  LogicExtractorFactory logic_extractor(sta_.get(), &logger_);
  logic_extractor.AppendEndpoint(flop_input_vertex);
  LogicCut cut = logic_extractor.BuildLogicCut(abc_library);

  EXPECT_EQ(cut.cut_instances().size(), 1);
  EXPECT_EQ(std::string(network->name(*cut.cut_instances().begin())), "_403_");
}

TEST_F(AbcTest, ExtractsEmptyCutSetCorrectly)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  LoadVerilog("empty_cut_set.v");

  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Vertex* flop_input_vertex = nullptr;
  for (sta::Vertex* vertex : *sta_->endpoints()) {
    if (std::string(vertex->name(network)) == "output_flop/D") {
      flop_input_vertex = vertex;
    }
  }
  EXPECT_NE(flop_input_vertex, nullptr);

  LogicExtractorFactory logic_extractor(sta_.get(), &logger_);
  logic_extractor.AppendEndpoint(flop_input_vertex);
  LogicCut cut = logic_extractor.BuildLogicCut(abc_library);

  EXPECT_TRUE(cut.IsEmpty());
}

TEST_F(AbcTest, ExtractSideOutputsCorrectly)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  LoadVerilog("side_outputs_extract.v");

  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Vertex* flop_input_vertex = nullptr;
  for (sta::Vertex* vertex : *sta_->endpoints()) {
    if (std::string(vertex->name(network)) == "output_flop/D") {
      flop_input_vertex = vertex;
    }
  }
  EXPECT_NE(flop_input_vertex, nullptr);

  LogicExtractorFactory logic_extractor(sta_.get(), &logger_);
  logic_extractor.AppendEndpoint(flop_input_vertex);
  LogicCut cut = logic_extractor.BuildLogicCut(abc_library);

  std::unordered_set<std::string> primary_output_names;
  for (sta::Net* net : cut.primary_outputs()) {
    primary_output_names.insert(network->name(net));
  }

  // Since a single net feeds both of these outputs should expect just 1 output
  EXPECT_EQ(cut.primary_outputs().size(), 1);
  EXPECT_THAT(primary_output_names, Contains("flop_net"));
}

TEST_F(AbcTest, BuildAbcMappedNetworkFromLogicCut)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  LoadVerilog("side_outputs_extract_logic_depth.v");

  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Vertex* flop_input_vertex = nullptr;
  for (sta::Vertex* vertex : *sta_->endpoints()) {
    if (std::string(vertex->name(network)) == "output_flop/D") {
      flop_input_vertex = vertex;
    }
  }
  EXPECT_NE(flop_input_vertex, nullptr);

  LogicExtractorFactory logic_extractor(sta_.get(), &logger_);
  logic_extractor.AppendEndpoint(flop_input_vertex);
  LogicCut cut = logic_extractor.BuildLogicCut(abc_library);

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> abc_network
      = cut.BuildMappedAbcNetwork(abc_library, network, &logger_);

  abc::Abc_NtkSetName(abc_network.get(), strdup("temp_network_name"));

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> logic_network(
      abc::Abc_NtkToLogic(abc_network.get()), &abc::Abc_NtkDelete);

  // Build map of primary output names to primary output indicies in ABC
  std::map<std::string, int> primary_output_name_to_index
      = AbcLogicNetworkNameToPrimaryOutputIds(logic_network.get());

  std::array<int, 2> input_vector = {1, 1};
  utl::UniquePtrWithDeleter<int> output_vector(
      abc::Abc_NtkVerifySimulatePattern(logic_network.get(),
                                        input_vector.data()),
      &free);

  // Both outputs are just the and gate.
  EXPECT_EQ(output_vector.get()[primary_output_name_to_index.at("flop_net")],
            0);  // Expect that !(1 & 1) == 0
  EXPECT_EQ(output_vector.get()[primary_output_name_to_index.at("and_output")],
            1);  // Expect that (1 & 1) == 1
}

TEST_F(AbcTest, BuildComplexLogicCone)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  LoadVerilog("aes_nangate45.v", /*top=*/"aes_cipher_top");

  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Vertex* flop_input_vertex = nullptr;
  for (sta::Vertex* vertex : *sta_->endpoints()) {
    if (std::string(vertex->name(network)) == "_32989_/D") {
      flop_input_vertex = vertex;
    }
  }
  EXPECT_NE(flop_input_vertex, nullptr);

  LogicExtractorFactory logic_extractor(sta_.get(), &logger_);
  logic_extractor.AppendEndpoint(flop_input_vertex);
  LogicCut cut = logic_extractor.BuildLogicCut(abc_library);

  EXPECT_NO_THROW(cut.BuildMappedAbcNetwork(abc_library, network, &logger_));
}

TEST_F(AbcTest, InsertingMappedLogicCutDoesNotThrow)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  LoadVerilog("aes_nangate45.v", /*top=*/"aes_cipher_top");

  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Vertex* flop_input_vertex = nullptr;
  for (sta::Vertex* vertex : *sta_->endpoints()) {
    if (std::string(vertex->name(network)) == "_33122_/D") {
      flop_input_vertex = vertex;
    }
  }
  EXPECT_NE(flop_input_vertex, nullptr);

  LogicExtractorFactory logic_extractor(sta_.get(), &logger_);
  logic_extractor.AppendEndpoint(flop_input_vertex);
  LogicCut cut = logic_extractor.BuildLogicCut(abc_library);

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_abc_network
      = cut.BuildMappedAbcNetwork(abc_library, network, &logger_);

  rmp::UniqueName unique_name;
  EXPECT_NO_THROW(cut.InsertMappedAbcNetwork(
      mapped_abc_network.get(), abc_library, network, unique_name, &logger_));
}

TEST_F(AbcTest, InsertingMappedLogicAfterOptimizationCutDoesNotThrow)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  LoadVerilog("aes_nangate45.v", /*top=*/"aes_cipher_top");

  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Vertex* flop_input_vertex = nullptr;
  for (sta::Vertex* vertex : *sta_->endpoints()) {
    if (std::string(vertex->name(network)) == "_33122_/D") {
      flop_input_vertex = vertex;
    }
  }
  EXPECT_NE(flop_input_vertex, nullptr);

  LogicExtractorFactory logic_extractor(sta_.get(), &logger_);
  logic_extractor.AppendEndpoint(flop_input_vertex);
  LogicCut cut = logic_extractor.BuildLogicCut(abc_library);

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_abc_network
      = cut.BuildMappedAbcNetwork(abc_library, network, &logger_);

  DelayOptimizationStrategy strat;
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> remapped
      = strat.Optimize(mapped_abc_network.get(), abc_library, &logger_);

  rmp::UniqueName unique_name;
  EXPECT_NO_THROW(cut.InsertMappedAbcNetwork(
      remapped.get(), abc_library, network, unique_name, &logger_));
}

TEST_F(AbcTest,
       AfterExtractingAndReinsertingCuttingAgainResultsInCorrectSimulation)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  LoadVerilog("side_outputs_extract_logic_depth.v");

  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Vertex* flop_input_vertex = nullptr;
  for (sta::Vertex* vertex : *sta_->endpoints()) {
    if (std::string(vertex->name(network)) == "output_flop/D") {
      flop_input_vertex = vertex;
    }
  }
  EXPECT_NE(flop_input_vertex, nullptr);

  LogicExtractorFactory logic_extractor(sta_.get(), &logger_);
  logic_extractor.AppendEndpoint(flop_input_vertex);
  LogicCut cut = logic_extractor.BuildLogicCut(abc_library);

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_abc_network
      = cut.BuildMappedAbcNetwork(abc_library, network, &logger_);

  rmp::UniqueName unique_name;
  cut.InsertMappedAbcNetwork(
      mapped_abc_network.get(), abc_library, network, unique_name, &logger_);

  // Re-extract the same cone, and try to simulate it to make sure everything
  // still simulates correctly
  LogicExtractorFactory logic_extractor_post_insert(sta_.get(), &logger_);
  logic_extractor_post_insert.AppendEndpoint(flop_input_vertex);
  LogicCut cut_post_insert
      = logic_extractor_post_insert.BuildLogicCut(abc_library);

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_abc_network_post_insert
      = cut.BuildMappedAbcNetwork(abc_library, network, &logger_);

  abc::Abc_NtkSetName(mapped_abc_network_post_insert.get(),
                      strdup("temp_network_name"));

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> logic_network(
      abc::Abc_NtkToLogic(mapped_abc_network_post_insert.get()),
      &abc::Abc_NtkDelete);

  // Build map of primary output names to primary output indicies in ABC
  std::map<std::string, int> primary_output_name_to_index
      = AbcLogicNetworkNameToPrimaryOutputIds(logic_network.get());

  std::array<int, 2> input_vector = {1, 1};
  utl::UniquePtrWithDeleter<int> output_vector(
      abc::Abc_NtkVerifySimulatePattern(logic_network.get(),
                                        input_vector.data()),
      &free);

  // Both outputs are just the and gate.
  EXPECT_EQ(output_vector.get()[primary_output_name_to_index.at("flop_net")],
            0);  // Expect that !(1 & 1) == 0
  EXPECT_EQ(output_vector.get()[primary_output_name_to_index.at("and_output")],
            1);  // Expect that (1 & 1) == 1
}

TEST_F(AbcTest, ResynthesisStrategyDoesNotThrow)
{
  LoadVerilog("aes_nangate45.v", /*top=*/"aes_cipher_top");

  UniqueName name_generator;
  ZeroSlackStrategy zero_slack;
  EXPECT_NO_THROW(
      zero_slack.OptimizeDesign(sta_.get(), name_generator, &logger_));
}

TEST_F(AbcTestSky130, EnsureThatSky130MultiOutputConstCellsAreMapped)
{
  AbcLibraryFactory factory(&logger_);
  factory.AddDbSta(sta_.get());
  AbcLibrary abc_library = factory.Build();

  LoadVerilog("sky130_const_cell.v");

  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Instance* flop_input_instance = network->findInstance("_403_");
  EXPECT_NE(flop_input_instance, nullptr);
  sta::Net* flop_net = network->findNet("flop_net");
  EXPECT_NE(flop_net, nullptr);

  std::vector<sta::Net*> primary_inputs = {};
  std::vector<sta::Net*> primary_outputs = {flop_net};
  sta::InstanceSet cut_instances(network);
  cut_instances.insert(flop_input_instance);
  LogicCut cut(primary_inputs, primary_outputs, cut_instances);

  // Create abc network that matches the underlying LogicCut
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> abc_network(
      abc::Abc_NtkAlloc(abc::Abc_NtkType_t::ABC_NTK_NETLIST,
                        abc::Abc_NtkFunc_t::ABC_FUNC_MAP,
                        /*fUseMemMan=*/1),
      &abc::Abc_NtkDelete);
  abc::Abc_NtkSetName(abc_network.get(), strdup("test_module"));

  abc::Mio_Library_t* mio_library
      = abc::Abc_SclDeriveGenlibSimple(abc_library.abc_library());
  abc_network->pManFunc = mio_library;

  abc::Abc_Obj_t* output = abc::Abc_NtkCreatePo(abc_network.get());
  abc::Abc_Obj_t* output_net = abc::Abc_NtkCreateNet(abc_network.get());

  abc::Abc_Obj_t* const_1 = abc::Abc_NtkCreateNode(abc_network.get());
  abc::Abc_ObjSetData(const_1, abc::Mio_LibraryReadConst1(mio_library));

  abc::Abc_ObjAddFanin(output, output_net);
  abc::Abc_ObjAddFanin(output_net, const_1);

  std::string output_name = "flop_net";
  abc::Abc_ObjAssignName(output_net, output_name.data(), /*pSuffix=*/nullptr);

  rmp::UniqueName unique_namer;

  // We want to make sure this thing correctly maps to the multi-output sky130
  // cell.
  cut.InsertMappedAbcNetwork(
      abc_network.get(), abc_library, network, unique_namer, &logger_);

  // Go searching for our const cell. It has a random name now.
  odb::dbSet<odb::dbInst> insts = db_->getChip()->getBlock()->getInsts();
  std::vector<odb::dbInst*> constant_cells;
  for (odb::dbInst* inst : insts) {
    odb::dbMaster* master = inst->getMaster();
    if (std::string(master->getName()) == "sky130_fd_sc_hd__conb_1") {
      constant_cells.push_back(inst);
    }
  }

  EXPECT_EQ(constant_cells.size(), 1);
  EXPECT_NE(std::string(constant_cells[0]->getName()), "_403_");
}

}  // namespace rmp
