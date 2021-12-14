package bfs

import chisel3._
import nf_arm_doce.{axidata, axilitedata}
import chisel3.util.Decoupled
import chisel3.util._
import numa.LookupTable

/*class BFS(AXI_ADDR_WIDTH : Int = 44, AXI_DATA_WIDTH: Int = 16, AXI_ID_WIDTH: Int = 18, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val config = new axilitedata(AXI_ADDR_WIDTH)
    val PLmempory = Flipped(new axidata(64, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val PSmempory = Flipped(new axidata(64, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))

  })

  //1st entry for start register
  val controls = Module(new LookupTable(depth = 2, AXI_ADDR_WIDTH = AXI_ADDR_WIDTH))
  controls.config <> io.config
  val start = controls.io.data(0)(0)


}*/
class xbar(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 8, AXI_ID_WIDTH: Int = 1, AXI_SIZE_WIDTH: Int = 3) extends BlackBox {
  val io = IO(new Bundle {
    val s_axi = new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 2)
    val m_axi = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val aclk = Input(Bool())
    val aresetn = Input(Bool())
  })
}

class memory(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 8, AXI_ID_WIDTH: Int = 4, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val data_out = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val count = Output(UInt(32.W))
    val base_addr = Input(UInt(64.W))
    //val data_in = (new axidata(64, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
  })

  val xbar = Module(new xbar(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
  xbar.io.aclk := clock.asBool()
  xbar.io.aresetn := ~reset.asBool()
  xbar.io.m_axi.rlast := io.data_out.rlast
  xbar.io.m_axi.rvalid := io.data_out.rvalid
  xbar.io.m_axi.rdata := io.data_out.rdata
  xbar.io.m_axi.rid := io.data_out.rid
  xbar.io.m_axi.rresp := io.data_out.rresp
  io.data_out.rready := xbar.io.m_axi.rready

  io.data_out.araddr := xbar.io.m_axi.araddr
  io.data_out.arvalid := xbar.io.m_axi.arvalid
  io.data_out.arid := xbar.io.m_axi.arid
  io.data_out.arsize := xbar.io.m_axi.arsize
  io.data_out.arlen := xbar.io.m_axi.arlen
  io.data_out.arlock := xbar.io.m_axi.arlock
  io.data_out.arburst := xbar.io.m_axi.arburst
  xbar.io.m_axi.arready := io.data_out.arready
  //read offset array
  val count = RegInit(0.U(32.W))
  val offset_arvalid = RegInit(false.B)
  when(offset_arvalid && xbar.io.s_axi.arready(1) && xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)){
    count := count
  }.elsewhen(offset_arvalid && xbar.io.s_axi.arready(1)) {
    count := count + 1.U
  }.elsewhen(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)) {
    count := count - 1.U
  }

  val full = (count >= 63.U)
  when(full.asBool()){
    offset_arvalid := false.B
  }.otherwise{
    offset_arvalid := true.B
  }

  io.count := count

  //read edge array
  val edge_count = RegInit(0.U(32.W))
  val edge_arvalid =  edge_count > 0.U
    when(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)
    && edge_arvalid && xbar.io.s_axi.arready(0)){
      edge_count := edge_count
    }.elsewhen(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)){
    edge_count := edge_count + 1.U
  }.elsewhen(edge_arvalid && xbar.io.s_axi.arready(0)) {
    edge_count := edge_count - 1.U
  }

  xbar.io.s_axi.arvalid := Cat(offset_arvalid & !full, edge_arvalid)
  xbar.io.s_axi.arid := Cat(count(AXI_ID_WIDTH-1, 0), edge_count(AXI_ID_WIDTH-1, 0))
  xbar.io.s_axi.araddr := Cat(io.base_addr, (io.base_addr + 0x10000.U))
  xbar.io.s_axi.arburst := Cat(1.U(2.W), 1.U(2.W))
  xbar.io.s_axi.arlen := Cat(0.U(8.W), 7.U(8.W))
  xbar.io.s_axi.arsize := Cat(log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W), log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W))
  xbar.io.s_axi.arlock := 0.U(2.W)

  xbar.io.s_axi.rready := Cat(1.U(1.W), 1.U(1.W))

  //write value array
  val wcount = RegInit(0.U(32.W))
  when(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)
    && io.data_out.wvalid.asBool() && io.data_out.wready.asBool()){
    wcount := wcount
  }.elsewhen(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)){
    wcount := wcount + 1.U
  }.elsewhen(io.data_out.wvalid.asBool() && io.data_out.wready.asBool()) {
    wcount := wcount - 1.U
  }

  io.data_out.wvalid := wcount > 0.U
  io.data_out.wlast := 1.U
  io.data_out.wdata := 0.U
  io.data_out.wstrb := VecInit(Seq.fill(AXI_DATA_WIDTH)(1.U(1.W))).asUInt()

  val awcount = RegInit(0.U(32.W))
  when(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)
    && io.data_out.awvalid.asBool() && io.data_out.awready.asBool()){
    awcount := awcount
  }.elsewhen(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)){
    awcount := awcount + 1.U
  }.elsewhen(io.data_out.awvalid.asBool() && io.data_out.awready.asBool()) {
    awcount := awcount - 1.U
  }

  val awaddr = RegInit(0.U(30.W))
  when(io.data_out.awvalid.asBool() && io.data_out.awready.asBool()) {
    awaddr := awaddr + 0x1000.U
  }

  io.data_out.awvalid := awcount > 0.U
  io.data_out.awid := awcount(AXI_ID_WIDTH-1, 0)
  io.data_out.awsize := log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W)
  io.data_out.awlen := 0.U
  io.data_out.awburst := 1.U(2.W)
  io.data_out.awlock := 0.U
  io.data_out.awaddr := io.base_addr + awaddr

  io.data_out.bready := 1.U
}

class embedding_mc(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 4, AXI_SIZE_WIDTH: Int = 3, BASE_ADDR : String = "x800000000") extends Module{
  val io = IO(new Bundle() {
    val data_out = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val count = Output(UInt(32.W))
    val base_addr = Input(UInt(64.W))
    //val data_in = (new axidata(64, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
  })
  assert(AXI_DATA_WIDTH >= 8)

  //read offset and edge embedding array
  val count = RegInit(0.U(32.W))
  val embedding_arvalid = RegInit(false.B)
  when(io.data_out.arvalid.asBool() && io.data_out.arready.asBool() && io.data_out.rvalid.asBool() && io.data_out.rready.asBool() && io.data_out.rlast.asBool()){
    count := count
  }.elsewhen(io.data_out.arvalid.asBool() && io.data_out.arready.asBool()) {
    count := count + 1.U
  }.elsewhen(io.data_out.rvalid.asBool() && io.data_out.rready.asBool() && io.data_out.rlast.asBool()) {
    count := count - 1.U
  }

  val full = (count >= 32.U)
  when(full.asBool()){
    embedding_arvalid := false.B
  }.otherwise{
    embedding_arvalid := true.B
  }

  val addr = RegInit(0.U(30.W))
  when(io.data_out.arvalid.asBool() && io.data_out.arready.asBool()) {
    addr := addr + 0x1000.U
  }
  io.count := count
  io.data_out.arvalid := embedding_arvalid & !full
  io.data_out.arid := count(AXI_ID_WIDTH-1, 0)
  io.data_out.arburst := 1.U(2.W)
  io.data_out.arlock := 0.U
  io.data_out.arlen := 0.U
  io.data_out.arsize := log2Ceil(AXI_DATA_WIDTH).U
  io.data_out.araddr := io.base_addr + addr

  io.data_out.rready := 1.U

  //write value array
  val wcount = RegInit(0.U(32.W))
  when(io.data_out.rvalid.asBool() && io.data_out.rready.asBool() && io.data_out.rlast.asBool()
    && io.data_out.wvalid.asBool() && io.data_out.wready.asBool()){
    wcount := wcount
  }.elsewhen(io.data_out.rvalid.asBool() && io.data_out.rready.asBool() && io.data_out.rlast.asBool()){
    wcount := wcount + 1.U
  }.elsewhen(io.data_out.wvalid.asBool() && io.data_out.wready.asBool()) {
    wcount := wcount - 1.U
  }

  io.data_out.wvalid := wcount > 0.U
  io.data_out.wlast := 1.U
  io.data_out.wdata := 0.U
  io.data_out.wstrb := 0xff.U

  val awcount = RegInit(0.U(32.W))
  when(io.data_out.rvalid.asBool() && io.data_out.rready.asBool() && io.data_out.rlast.asBool()
    && io.data_out.awvalid.asBool() && io.data_out.awready.asBool()){
    awcount := awcount
  }.elsewhen(io.data_out.rvalid.asBool() && io.data_out.rready.asBool() && io.data_out.rlast.asBool()){
    awcount := awcount + 1.U
  }.elsewhen(io.data_out.awvalid.asBool() && io.data_out.awready.asBool()) {
    awcount := awcount - 1.U
  }

  val awaddr = RegInit(0.U(30.W))
  when(io.data_out.awvalid.asBool() && io.data_out.awready.asBool()) {
    awaddr := awaddr + 0x1000.U
  }

  io.data_out.awvalid := awcount > 0.U
  io.data_out.awid := awcount(AXI_ID_WIDTH-1, 0)
  io.data_out.awsize := log2Ceil(8).U
  io.data_out.awlen := 0.U
  io.data_out.awburst := 1.U(2.W)
  io.data_out.awlock := 0.U
  io.data_out.awaddr := io.base_addr + awaddr

  io.data_out.bready := 1.U
}