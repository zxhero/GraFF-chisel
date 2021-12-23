package bfs

import chisel3._
import chisel3.experimental.ChiselEnum
import nf_arm_doce.{axidata, axilitedata, streamdata}
import chisel3.util.Decoupled
import chisel3.util._
import utils._
import numa.LookupTable

/*class memory(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 8, AXI_ID_WIDTH: Int = 4, AXI_SIZE_WIDTH: Int = 3) extends Module{
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
}*/

class axiar(AXI_ADDR_WIDTH : Int = 44, AXI_ID_WIDTH: Int = 18, AXI_SIZE_WIDTH: Int = 3, NUM : Int = 1) extends Bundle {
  val araddr = Input(UInt((NUM * AXI_ADDR_WIDTH).W))
  val arid = Input(UInt((NUM * AXI_ID_WIDTH).W))
  val arlen = Input(UInt((NUM * 8).W))
  val arsize = Input(UInt((NUM * AXI_SIZE_WIDTH).W))
  val arburst = Input(UInt((NUM * 2).W))
  val arlock = Input(UInt((NUM * 1).W))
  val arvalid = Input(UInt((NUM * 1).W))
  val arready = Output(UInt((NUM * 1).W))
}

class axiaw(AXI_ADDR_WIDTH : Int = 44, AXI_ID_WIDTH: Int = 18, AXI_SIZE_WIDTH: Int = 3, NUM : Int = 1) extends Bundle {
  val awaddr = Input(UInt((NUM * AXI_ADDR_WIDTH).W))
  val awid = Input(UInt((NUM * AXI_ID_WIDTH).W))
  val awlen = Input(UInt((NUM * 8).W))
  val awsize = Input(UInt((NUM * AXI_SIZE_WIDTH).W))
  val awburst = Input(UInt((NUM * 2).W))
  val awlock = Input(UInt((NUM * 1).W))
  val awvalid = Input(UInt((NUM * 1).W))
  val awready = Output(UInt((NUM * 1).W))
}

class axiw(AXI_DATA_WIDTH: Int = 16, NUM : Int = 1) extends Bundle {
  val wdata = Input(UInt((NUM*8*AXI_DATA_WIDTH).W))
  val wstrb = Input(UInt((NUM*AXI_DATA_WIDTH).W))
  val wlast = Input(UInt((NUM * 1).W))
  val wvalid = Input(UInt((NUM * 1).W))
  val wready = Output(UInt((NUM * 1).W))
}

class axib(AXI_ID_WIDTH: Int = 18, NUM : Int = 1) extends Bundle {
  val bresp = Output(UInt((NUM * 2).W))
  val bid = Output(UInt((NUM * AXI_ID_WIDTH).W))
  val bvalid = Output(UInt((NUM * 1).W))
  val bready = Input(UInt((NUM * 1).W))
}

class axir(AXI_DATA_WIDTH: Int = 16, AXI_ID_WIDTH: Int = 18, NUM : Int = 1) extends Bundle{
  val rdata = Output(UInt((NUM*8*AXI_DATA_WIDTH).W))
  val rid = Output(UInt((NUM * AXI_ID_WIDTH).W))
  val rlast = Output(UInt((NUM * 1).W))
  val rresp = Output(UInt((NUM * 2).W))
  val rvalid = Output(UInt((NUM * 1).W))
  val rready = Input(UInt((NUM * 1).W))
}

class axi_arbitrator(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 4, AXI_SIZE_WIDTH: Int = 3, NUM : Int = 2) extends  Module {
  val io = IO(new Bundle() {
    val xbar_in = (new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, NUM))
    val ddr_out = Flipped(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
  })

  io.ddr_out.arburst := 1.U(2.W)
  io.ddr_out.arlock := 0.U
  io.ddr_out.arsize := log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W)

  val addr = RegInit(0.U((AXI_ADDR_WIDTH * NUM).W))
  val id = RegInit(0.U((NUM * AXI_ID_WIDTH).W))
  val valid = RegInit(0.U((NUM).W))
  val len = RegInit(0.U((NUM * 8).W))
  when(io.xbar_in.arvalid =/= 0.U && io.xbar_in.arready =/= 0.U){
    addr := io.xbar_in.araddr
    id := io.xbar_in.arid
    len := io.xbar_in.arlen
  }
  io.xbar_in.arready := VecInit(Seq.fill(NUM)(~(valid.orR()))).asUInt()

  val select = MuxCase(0.U,
    Array.tabulate(NUM)(x => (valid(x) -> (1.U(NUM.W) << x).asUInt()))
  )
  io.ddr_out.arvalid := select.orR()
  io.ddr_out.arlen := Mux1H(Seq.tabulate(NUM)(x => (select(x) -> len(x * 8 + 7, x * 8))))
  io.ddr_out.arid := Mux1H(Seq.tabulate(NUM)(x => (select(x) -> id(x * AXI_ID_WIDTH + AXI_ID_WIDTH - 1, x * AXI_ID_WIDTH))))
  io.ddr_out.araddr := Mux1H(Seq.tabulate(NUM)(x => (select(x) -> addr(x * AXI_ADDR_WIDTH + AXI_ADDR_WIDTH - 1, x * AXI_ADDR_WIDTH))))

  when(io.xbar_in.arvalid =/= 0.U && io.xbar_in.arready =/= 0.U){
    valid := io.xbar_in.arvalid
  }.elsewhen(io.ddr_out.arvalid.asBool() && io.ddr_out.arready.asBool()) {
    valid := valid ^ select
  }
}

class URAM(size : Int = 1024, width : Int = 32) extends BlackBox{
  val io = IO(new Bundle() {
    val addra = Input(UInt(log2Ceil(size).W))
    val clka = Input(Bool())
    val dina = Input(UInt(width.W))
    val douta = Input(UInt(width.W))
    val ena = Input(Bool())
    val wea = Input(Bool())
    val addrb = Input(UInt(log2Ceil(size).W))
    val clkb = Input(Bool())
    val dinb = Input(UInt(width.W))
    val doutb = Input(UInt(width.W))
    val enb = Input(Bool())
    val web = Input(Bool())
  })
}

class WB_engine(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val ddr_aw = Flipped(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Flipped(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(new axib(AXI_ID_WIDTH, 1))

    val xbar_in = (new streamdata(4))
    val level_base_addr = Input(UInt(64.W))
    val level_size = Input(UInt(64.W))     //2^level_size in bytes
    val level = Input(UInt(32.W))
  })

  val buffer = Module(new URAM(524288,32))

  //update buffer
  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val read_size  = Value(0x1.U) // i "load"  -> 000_0011
    val write_addr   = Value(0x2.U) // i "imm"   -> 001_0011
    val write_level = Value(0x3.U)
    val write_size = Value(0x4.U)
    val read_addr = Value(0x5.U)

    val read_level1 = Value(0x6.U)
    val wb_level1 = Value(0x7.U)
    val read_level2 = Value(0x8.U)
    val wb_level2 = Value(0x9.U)
  }
  val update_sm = RegInit(sm.read_size)
  val wb_sm = RegInit(sm.idole)
  val vid = RegInit(0.U(32.W))
  val level = RegInit(0.U(32.W))
  when(io.xbar_in.tvalid && io.xbar_in.tready){
    vid := io.xbar_in.tdata
    level := io.level
  }
  io.xbar_in.tready := update_sm === sm.read_size & (wb_sm === sm.idole)
  when(io.xbar_in.tvalid && io.xbar_in.tready){
    update_sm := sm.read_addr
  }.elsewhen(update_sm === sm.read_addr){
    update_sm := sm.write_addr
  }.elsewhen(update_sm === sm.write_addr){
    update_sm := sm.write_level
  }.elsewhen(update_sm === sm.write_level){
    update_sm := sm.write_size
  }.elsewhen(update_sm === sm.write_size){
    update_sm := sm.read_size
  }
  val addr = Wire(UInt(16.W))
  addr := vid(11, 0) << 2.U
  val block_addr = vid(25 - 2, 14 - 2) << 7
  val size_addr = 0.U(7.W)
  val size = RegInit(0.U(32.W))
  val addr_addr = 2.U + size >> 1.U
  val level_addr = (2 + 42).U + size
  when(update_sm === sm.read_size) {
    size := buffer.io.douta
  }
  val old_addr_pair = RegInit(0.U(32.W))
  when(update_sm === sm.read_addr) {
    old_addr_pair := buffer.io.douta
  }
  val new_addr_pair = Mux(size(0), Cat(addr, old_addr_pair(15, 0)), Cat(old_addr_pair(31, 16), addr))
  buffer.io.ena := (io.xbar_in.tvalid & io.xbar_in.tready) | (update_sm === sm.read_addr) | (update_sm === sm.write_addr) |
    (update_sm === sm.write_level) | (update_sm === sm.write_size)
  buffer.io.wea := (update_sm === sm.write_addr) | (update_sm === sm.write_level) | (update_sm === sm.write_size)
  buffer.io.addra := Mux1H(Seq(
    (update_sm === sm.read_size) -> (block_addr.asUInt() + size_addr),
    (update_sm === sm.read_addr) -> (block_addr.asUInt() + addr_addr),
    (update_sm === sm.write_addr) -> (block_addr.asUInt() + addr_addr),
    (update_sm === sm.write_level) -> (block_addr.asUInt() + level_addr),
    (update_sm === sm.write_size) -> (block_addr.asUInt() + size_addr)
  ))
  buffer.io.dina := Mux1H(Seq(
    (update_sm === sm.write_addr) -> (new_addr_pair),
    (update_sm === sm.write_level) -> (level),
    (update_sm === sm.write_size) -> (size + 1.U)
  ))
  buffer.io.clka := clock.asBool()

  //write back buffer
  val count = RegInit(0.U(8.W))
  val aw_buffer = Module(new fifo(32, AXI_ADDR_WIDTH))
  val w_buffer = Module(new fifo(32, 32))
  val wb_block_addr = RegInit(0.U(19.W))
  when(update_sm === sm.write_size && buffer.io.dina === 84.U){
    wb_block_addr := block_addr
  }
  when(update_sm === sm.write_size && buffer.io.dina === 84.U){
    wb_sm := sm.read_addr
  }.elsewhen(wb_sm === sm.read_addr) {
    wb_sm := sm.read_level1
  }.elsewhen(wb_sm === sm.read_level1) {
    wb_sm := sm.wb_level1
  }.elsewhen(wb_sm === sm.wb_level1 && aw_buffer.io.full === false.B && w_buffer.io.full === false.B) {
    wb_sm := sm.read_level2
  }.elsewhen(wb_sm === sm.read_level2) {
    wb_sm := sm.wb_level2
  }.elsewhen(wb_sm === sm.wb_level2 && aw_buffer.io.full === false.B && w_buffer.io.full === false.B) {
    when(count === 83.U){
      wb_sm := sm.write_size
    }.otherwise{
      wb_sm := sm.read_addr
    }
  }.elsewhen(wb_sm === sm.write_size) {
    wb_sm := sm.idole
  }
  when(update_sm === sm.write_size && buffer.io.dina === 84.U){
    count := 0.U
  }.elsewhen(wb_sm === sm.wb_level1 && aw_buffer.io.full === false.B && w_buffer.io.full === false.B){
    count := count + 1.U
  }.elsewhen(wb_sm === sm.wb_level2 && aw_buffer.io.full === false.B && w_buffer.io.full === false.B) {
    count := count + 1.U
  }

  val wb_addr_pair = RegInit(0.U(32.W))
  val wb_level = RegInit(0.U(32.W))
  buffer.io.enb := (wb_sm === sm.read_addr) | (wb_sm === sm.read_level1) | (wb_sm === sm.read_level2) |
    (wb_sm === sm.write_size)
  buffer.io.web := (wb_sm === sm.write_size)
  buffer.io.addrb := Mux1H(Seq(
    (update_sm === sm.read_addr) -> (wb_block_addr.asUInt() + 2.U + (count >> 1.U)),
    (update_sm === sm.read_level1) -> (wb_block_addr.asUInt() + (2 + 42).U + count),
    (update_sm === sm.read_level2) -> (wb_block_addr.asUInt() + (2 + 42).U + count),
    (update_sm === sm.write_size) -> (wb_block_addr.asUInt() + size_addr)
  ))
  buffer.io.dinb := 0.U
  buffer.io.clkb := clock.asBool()
  when(wb_sm === sm.read_addr){
    wb_addr_pair := buffer.io.doutb
  }
  when(wb_sm === sm.read_level1 || wb_sm === sm.read_level2){
    wb_level := buffer.io.doutb
  }

  aw_buffer.io.writeFlag := wb_sm === sm.wb_level1 || wb_sm === sm.wb_level2
  aw_buffer.io.dataIn := Mux(wb_sm === sm.wb_level1, io.level_base_addr + Cat(wb_block_addr(18, 7), wb_addr_pair(13,0)).asUInt(),
    io.level_base_addr + Cat(wb_block_addr(18, 7), wb_addr_pair(16 + 14 - 1,16)))
  io.ddr_aw.awaddr := aw_buffer.io.dataOut
  io.ddr_aw.awlock := 0.U
  io.ddr_aw.awid := aw_buffer.io.rptr
  io.ddr_aw.awlen := 0.U
  io.ddr_aw.awburst := 1.U(2.W)
  io.ddr_aw.awsize := 2.U
  io.ddr_aw.awvalid := aw_buffer.io.empty === false.B
  aw_buffer.io.readFlag := io.ddr_aw.awready

  w_buffer.io.writeFlag := wb_sm === sm.wb_level1 || wb_sm === sm.wb_level2
  w_buffer.io.dataIn := wb_level
  io.ddr_w.wdata := w_buffer.io.dataOut
  io.ddr_w.wlast := true.B
  io.ddr_w.wvalid := w_buffer.io.empty === false.B
  io.ddr_w.wstrb := 0xf.U
  w_buffer.io.readFlag := io.ddr_w.wready

  io.ddr_b.bready := true.B
}

class Apply(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle(){
    val ddr_aw = Flipped(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Flipped(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(new axib(AXI_ID_WIDTH, 1))

    val gather_in = (new streamdata(4))

    val level = Input(UInt(32.W))
    val level_base_addr = Input(UInt(64.W))
    val level_size = Input(UInt(64.W))     //2^level_size in bytes
    val write_finish = Output(Bool())
  })

  //write value array
  val vertex_update_buffer = Module(new fifo(32, 64))
  vertex_update_buffer.io.writeFlag := io.gather_in.tvalid
  vertex_update_buffer.io.dataIn := Cat(io.gather_in.tdata, io.level)

  io.gather_in.tready := vertex_update_buffer.io.full === false.B

  val update_engine = Module(new WB_engine(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_DATA_WIDTH))
  update_engine.io.xbar_in.tdata := vertex_update_buffer.io.dataOut
  update_engine.io.xbar_in.tvalid := vertex_update_buffer.io.empty === false.B
  update_engine.io.level_base_addr := io.level_base_addr
  update_engine.io.level_size := io.level_size
  update_engine.io.xbar_in.tlast := true.B
  update_engine.io.xbar_in.tkeep := 0xff.U
  update_engine.io.level := io.level
  vertex_update_buffer.io.readFlag := update_engine.io.xbar_in.tready

  io.ddr_aw <> update_engine.io.ddr_aw
  io.ddr_w <> update_engine.io.ddr_w
  io.ddr_b <> update_engine.io.ddr_b

  io.write_finish := vertex_update_buffer.io.empty
}

class axis_arbitrator(AXIS_DATA_WIDTH: Int = 4, NUM : Int = 16, ELEMENT_WIDTH: Int = 4) extends Module{
  val io = IO(new Bundle() {
    val xbar_in = (new streamdata(AXIS_DATA_WIDTH * NUM, ELEMENT_WIDTH))
    val ddr_out = Flipped(new streamdata(AXIS_DATA_WIDTH, ELEMENT_WIDTH))
  })

  val data = RegInit(0.U((AXIS_DATA_WIDTH * NUM * 8).W))
  val keep = RegInit(0.U((AXIS_DATA_WIDTH * NUM / ELEMENT_WIDTH).W))

  when(io.xbar_in.tvalid && io.xbar_in.tready){
    data := io.xbar_in.tdata
  }
  io.xbar_in.tready := ~(keep.orR())

  val select = MuxCase(0.U, /*Array(
    keep(0) -> 1.U,
    keep(1) -> 2.U,
    keep(2) -> 4.U,
    keep(3) -> 8.U,
    keep(4) -> 0x0010.U,
    keep(5) -> 0x0020.U,
    keep(6) -> 0x0040.U,
    keep(7) -> 0x0080.U,
    keep(8) -> 0x0100.U,
    keep(9) -> 0x0200.U,
    keep(10) -> 0x0400.U,
    keep(11) -> 0x0800.U,
    keep(12) -> 0x1000.U,
    keep(13) -> 0x2000.U,
    keep(14) -> 0x4000.U,
    keep(15) -> 0x8000.U
  )*/
  Array.tabulate(NUM)(x => (keep((x+1) * (AXIS_DATA_WIDTH / ELEMENT_WIDTH) - 1, x * (AXIS_DATA_WIDTH / ELEMENT_WIDTH)).orR() -> (1.U(NUM.W) << x).asUInt())))
  io.ddr_out.tvalid := select.orR()
  io.ddr_out.tkeep := Mux1H(
    Seq.tabulate(NUM)(x => (select(x) -> keep((x + 1) * (AXIS_DATA_WIDTH / ELEMENT_WIDTH) - 1, x * (AXIS_DATA_WIDTH / ELEMENT_WIDTH))))
  )
  io.ddr_out.tlast := true.B
  io.ddr_out.tdata := /*Mux1H(Seq(
    select(0) -> data(31, 0),
    select(1) -> data(63, 32),
    select(2) -> data(3 * 32 - 1, 2 * 32),
    select(3) -> data(4 * 32 - 1, 3 * 32),
    select(4) -> data(5 * 32 - 1, 4 * 32),
    select(5) -> data(6 * 32 - 1, 5 * 32),
    select(6) -> data(7 * 32 - 1, 6 * 32),
    select(7) -> data(8 * 32 - 1, 7 * 32),
    select(8) -> data(9 * 32 - 1, 8 * 32),
    select(9) -> data(10 * 32 - 1, 9 * 32),
    select(10) -> data(11 * 32 - 1, 10 * 32),
    select(11) -> data(12 * 32 - 1, 11 * 32),
    select(12) -> data(13 * 32 - 1, 12 * 32),
    select(13) -> data(14 * 32 - 1, 13 * 32),
    select(14) -> data(15 * 32 - 1, 14 * 32),
    select(15) -> data(16 * 32 - 1, 15 * 32)
  ))*/
  Mux1H(Seq.tabulate(NUM)(x => (select(x) -> data((x + 1) * AXIS_DATA_WIDTH * 8 - 1, x * AXIS_DATA_WIDTH * 8))))

  val next_keep = Wire(Vec(NUM, UInt((AXIS_DATA_WIDTH / ELEMENT_WIDTH).W)))
  next_keep.zipWithIndex.map{
    case(k, i) => (k := Mux(select(i), 0.U((AXIS_DATA_WIDTH / ELEMENT_WIDTH).W), ~0.U((AXIS_DATA_WIDTH / ELEMENT_WIDTH).W)))
  }
  when(io.xbar_in.tvalid && io.xbar_in.tready){
    keep := io.xbar_in.tkeep
  }.elsewhen(io.ddr_out.tvalid && io.ddr_out.tready) {
    keep := keep & next_keep.asUInt()
  }
}

class Gather(AXI_DATA_WIDTH: Int = 64) extends Module{
  val io = IO(new Bundle() {
    val ddr_in = (new streamdata(AXI_DATA_WIDTH, 4))
    //val ddr_r = Flipped(new axir(AXI_DATA_WIDTH, AXI_ID_WIDTH))
    //val tier_base_addr = Input(UInt(AXI_ADDR_WIDTH.W))
    //val tier_count = Input(UInt(32.W))
    //val step_start = Input(Bool())

    val gather_out = Flipped(new streamdata(4, 4))
  })

  //front end of upward
  val vertex_in_buffer = Module(new fifo(32, 512 + 16))
  vertex_in_buffer.io.dataIn := Cat(io.ddr_in.tdata, io.ddr_in.tkeep)
  vertex_in_buffer.io.writeFlag := io.ddr_in.tvalid
  io.ddr_in.tready := vertex_in_buffer.io.full === false.B

  val Selector = Module(new axis_arbitrator(4, 16, 4))
  Selector.io.xbar_in.tdata := vertex_in_buffer.io.dataOut(512 + 16 - 1, 16)
  Selector.io.xbar_in.tvalid := vertex_in_buffer.io.empty === false.B
  Selector.io.xbar_in.tlast := true.B
  Selector.io.xbar_in.tkeep := vertex_in_buffer.io.dataOut(15, 0)
  vertex_in_buffer.io.readFlag := Selector.io.xbar_in.tready
  io.gather_out <> Selector.io.ddr_out
}

class Broadcast(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3, EMBEDDING : Int = 14) extends Module{
  val io = IO(new Bundle() {
    //val ddr_out = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val ddr_ar = Flipped(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val ddr_r = Flipped(new axir(AXI_DATA_WIDTH, AXI_ID_WIDTH))
    //val count = Output(UInt(32.W))
    val embedding_base_addr = Input(UInt(64.W))
    val edge_base_addr = Input(UInt(64.W))
    //val level_base_addr = Input(UInt(64.W))
    val gather_in = (new streamdata(4, 4))
    val xbar_out = Flipped(new streamdata(64, 4))
    //val level = Input(UInt(32.W))
    val read_finish = Output(Bool())

  })
  assert(AXI_DATA_WIDTH >= 8)
  assert(AXI_ID_WIDTH > log2Ceil(32))

  //back end of upward
  //read offset and edge embedding array
  val vertex_read_buffer = Module(new fifo(32, 32))
  vertex_read_buffer.io.writeFlag := io.gather_in.tvalid
  vertex_read_buffer.io.dataIn := io.gather_in.tdata
  io.gather_in.tready := vertex_read_buffer.io.full === false.B

  //read edge array
  val edge_read_buffer = Module(new fifo(32, 64))
  edge_read_buffer.io.dataIn := io.ddr_r.rdata(8 * 8 - 1, 0)
  edge_read_buffer.io.writeFlag := !io.ddr_r.rid(AXI_ID_WIDTH - 1) & io.ddr_r.rvalid & (io.ddr_r.rdata(31, 0) > EMBEDDING.asUInt())

  val ar_arbitator = Module(new axi_arbitrator(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 2))
  io.ddr_ar <> ar_arbitator.io.ddr_out

  ar_arbitator.io.xbar_in.araddr:= Cat(io.edge_base_addr + edge_read_buffer.io.dataOut(63,32) << 2 ,
    io.embedding_base_addr + vertex_read_buffer.io.dataOut << log2Ceil(4 * EMBEDDING + 8))
  ar_arbitator.io.xbar_in.arvalid := Cat(edge_read_buffer.io.empty === false.B, vertex_read_buffer.io.empty === false.B)
  val remainning_edges = (edge_read_buffer.io.dataOut(31, 0) - EMBEDDING.asUInt())
  //we assume number of edges of a vertex is less than 4096 + 14 for now. TODO
  val arlen = Mux(((remainning_edges >> log2Ceil(AXI_DATA_WIDTH/4)) << log2Ceil(AXI_DATA_WIDTH/4)).asUInt() < remainning_edges,
    (remainning_edges >> log2Ceil(AXI_DATA_WIDTH/4)).asUInt() + 1.U,
    (remainning_edges >> log2Ceil(AXI_DATA_WIDTH/4)))
  ar_arbitator.io.xbar_in.arlen := Cat(arlen.asUInt() - 1.U, ((4 * EMBEDDING + 8) / AXI_DATA_WIDTH - 1).asUInt())
  ar_arbitator.io.xbar_in.arburst := Cat(1.U(2.W), 1.U(2.W))
  ar_arbitator.io.xbar_in.arlock := 0.U
  ar_arbitator.io.xbar_in.arsize := Cat(log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W),
    log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W))
  ar_arbitator.io.xbar_in.arid := Cat((1.U(AXI_ID_WIDTH) << (AXI_ID_WIDTH - 1)).asUInt() + edge_read_buffer.io.rptr,
    vertex_read_buffer.io.rptr)
  vertex_read_buffer.io.readFlag := ar_arbitator.io.xbar_in.arready(0)
  edge_read_buffer.io.readFlag := ar_arbitator.io.xbar_in.arready(1)

  //front end of down ward
  val num_regfile = Module(new regFile(32, 32))
  num_regfile.io.writeFlag := ar_arbitator.io.xbar_in.arready(1) & ar_arbitator.io.xbar_in.arvalid(1)
  num_regfile.io.wptr := edge_read_buffer.io.rptr
  num_regfile.io.dataIn := remainning_edges
  num_regfile.io.rptr := io.ddr_r.rid(AXI_ID_WIDTH-2, 0)

  //back end of down ward
  object sm extends ChiselEnum {
    //val idole = Value(0x0.U)
    val firstBurst  = Value(0x1.U) // i "load"  -> 000_0011
    val remainingBurst   = Value(0x2.U) // i "imm"   -> 001_0011
  }
  val status = RegInit(sm.firstBurst)
  val num = RegInit(0.U(32.W))
  when(status === sm.firstBurst && io.ddr_r.rvalid.asBool() && io.ddr_r.rready.asBool()){
    when(io.ddr_r.rlast.asBool()){
      status := sm.firstBurst
      num := 0.U
    }.otherwise{
      num := num_regfile.io.dataOut
      status := sm.remainingBurst
    }
  }.elsewhen(status === sm.remainingBurst && io.ddr_r.rvalid.asBool() && io.ddr_r.rready.asBool()){
    when(io.ddr_r.rlast.asBool()){
      status := sm.firstBurst
      num := 0.U
    }.otherwise{
      num := num - 16.U
      status := sm.remainingBurst
    }
  }.otherwise{
    num := num
    status := status
  }
  val keep = Wire(Vec(16, Bool()))
  keep.zipWithIndex.map {
    case(k, i) => k := Mux(io.ddr_r.rid(AXI_ID_WIDTH-1), Mux(status === sm.firstBurst, num_regfile.io.dataOut > i.U, num > i.U),
      Mux(i.U < 2.U, false.B, true.B))
  }

  val vertex_out_fifo = Module(new fifo(2, 512 + 16))
  io.xbar_out.tvalid := vertex_out_fifo.io.empty === false.B
  io.xbar_out.tkeep := vertex_out_fifo.io.dataOut(15, 0)
  io.xbar_out.tdata := vertex_out_fifo.io.dataOut(512 + 16 - 1, 16)
  io.xbar_out.tlast := true.B
  vertex_out_fifo.io.readFlag := io.xbar_out.tready
  vertex_out_fifo.io.writeFlag := io.ddr_r.rvalid
  vertex_out_fifo.io.dataIn := Cat(io.ddr_r.rdata, keep.asUInt())
  io.ddr_r.rready := vertex_out_fifo.io.full === false.B

  io.read_finish := vertex_read_buffer.io.empty & edge_read_buffer.io.empty
}

/*
* PE ID is global addressable
* MC ID is local addressable
* */
class broadcast_xbar(AXIS_DATA_WIDTH: Int = 64, SLAVE_NUM: Int, MASTER_NUM: Int) extends Module {
  val io = IO(new Bundle {
    val ddr_in = Vec(MASTER_NUM, new streamdata(AXIS_DATA_WIDTH, 4))
    //val ddr_out = Flipped(Vec(MASTER_NUM, (new streamdata(AXIS_DATA_WIDTH, 4))))
    val pe_out = Flipped(Vec(SLAVE_NUM, (new streamdata(AXIS_DATA_WIDTH, 4))))
    //val pe_in = Vec(SLAVE_NUM, (new streamdata(4, 4)))
  })
  //ddr_in to pe_out direction
  val arbitrator = Module(new axis_arbitrator(AXIS_DATA_WIDTH, MASTER_NUM, 4))
  val ddr_data = Wire(Vec(MASTER_NUM, UInt((8 * AXIS_DATA_WIDTH).W)))
  val ddr_keep = Wire(Vec(MASTER_NUM, UInt((AXIS_DATA_WIDTH / 4).W)))
  val ddr_tvalid = Wire(Vec(MASTER_NUM, Bool()))
  io.ddr_in.zipWithIndex.map{
    case(ddr, i) => {
      ddr_tvalid(i) := ddr.tvalid
      ddr_keep(i) := Mux(ddr.tvalid, ddr.tkeep, 0.U)
      ddr_data(i) := ddr.tdata
      ddr.tready := arbitrator.io.xbar_in.tready
    }
  }
  arbitrator.io.xbar_in.tkeep := ddr_keep.asUInt()
  arbitrator.io.xbar_in.tdata := ddr_data.asUInt()
  arbitrator.io.xbar_in.tlast := true.B
  arbitrator.io.xbar_in.tvalid := ddr_tvalid.asUInt().orR()

  val PES_tready = Wire(Vec(SLAVE_NUM, Bool()))
  io.pe_out.zipWithIndex.map{
    case(pe, i) => {
      pe.tvalid := arbitrator.io.ddr_out.tvalid
      pe.tkeep := arbitrator.io.ddr_out.tkeep
      pe.tdata := arbitrator.io.ddr_out.tdata
      pe.tlast := arbitrator.io.ddr_out.tlast
      PES_tready(i) := pe.tready
    }
  }
  arbitrator.io.ddr_out.tready := PES_tready.asUInt().andR()
}

/*
* test_finish: PE has sent all current tier
* end: there is no current tier for the level
* start: the start signal for BFS from the root
* */
class Scatter(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 4, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends BlackBox {
  val io = IO(new Bundle() {
    val xbar_in = new streamdata(64, 4)
    val ddr_out = Flipped(new streamdata(4, 4))
    //val test_finish = Output(Bool())
    //val end = Output(Bool())
    //val start = Input(Bool())
    //val level = Input(UInt(32.W))
    //val root = Input(UInt(32.W))
  })
}

class multi_port_mc(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends BlackBox{
  val io = IO(new Bundle() {
    val cacheable_out = Flipped(new streamdata(AXI_DATA_WIDTH, 4))
    val cacheable_in = Vec(16, (new streamdata(4, 4)))
    val non_cacheable_in = new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)

    val ddr_out = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
  })
  /*val counter = RegInit(0.U(32.W))
  val addr = RegInit(0.U(AXI_ADDR_WIDTH.W))
  when(io.step_start){
    counter := io.tier_count
  }.elsewhen(io.ddr_ar.arvalid.asBool() & io.ddr_ar.arready.asBool()){
    counter := Mux(counter > (AXI_DATA_WIDTH / 4).U, counter - (AXI_DATA_WIDTH / 4).U, 0.U)
  }
  when(io.step_start){
    addr := io.tier_base_addr
  }.elsewhen(io.ddr_ar.arvalid.asBool() & io.ddr_ar.arready.asBool()){
    addr := addr + AXI_DATA_WIDTH.U
  }.elsewhen(counter === 0.U){
    addr := 0.U
  }
  io.ddr_ar.araddr := addr
  io.ddr_ar.arid := 0.U
  io.ddr_ar.arlen := 0.U
  io.ddr_ar.arburst := 1.U
  io.ddr_ar.arsize := 6.U
  io.ddr_ar.arlock := 0.U
  io.ddr_ar.arlen := 0.U
  io.ddr_ar.arvalid := counter > 0.U
  val rcounter = RegInit(0.U(32.W))
  when(io.step_start){
    rcounter := io.tier_count
  }.elsewhen(io.ddr_r.rvalid.asBool() & io.ddr_r.rready.asBool()){
    rcounter := Mux(rcounter > (AXI_DATA_WIDTH / 4).U, rcounter - (AXI_DATA_WIDTH / 4).U, 0.U)
  }
  val keep = Wire(Vec(16, Bool()))
  keep.zipWithIndex.map {
    case(k, i) => k := rcounter > i.U
  }

  //pe_in to ddr_out direction
  def vid_to_mcid(vid: UInt): UInt = {
    vid
  }
  val PEs_tvalid = Wire(Vec(MASTER_NUM, Vec(SLAVE_NUM, Bool())))
  val PEs_tkeep = Wire(Vec(MASTER_NUM, Vec(SLAVE_NUM, Bool())))
  val PEs_tdata = Wire(Vec(SLAVE_NUM, UInt(32.W)))
  io.pe_in.zipWithIndex.map{
    case(pe, i) => {
      PEs_tkeep.zipWithIndex.map{
        case (k, j) => k(i) := Mux(j.U === vid_to_mcid(pe.tdata), pe.tkeep.asBool(), false.B)
      }
      PEs_tvalid.zipWithIndex.map{
        case (v, j) => v(i) := Mux(j.U === vid_to_mcid(pe.tdata), pe.tvalid, false.B)
      }
      PEs_tdata(i) := pe.tdata
      pe.tready := io.ddr_out(vid_to_mcid((pe.tdata))).tready
    }
  }
  io.ddr_out.zipWithIndex.map{
    case(ddr, i) => {
      ddr.tkeep := PEs_tkeep(i).asUInt() & PEs_tvalid(i).asUInt()
      ddr.tvalid := PEs_tvalid(i).reduce(_|_)
      ddr.tlast := true.B
      ddr.tdata := PEs_tdata.asUInt()
    }
  }*/
}

class BFS(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val config = new axilitedata(AXI_ADDR_WIDTH)
    val PLmemory = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    //val PSmempory = Flipped(Vec(4, new axidata(64, 16, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))
  })

  //0 --- start register
  //1 --- embedding_base_addr(31, 0)
  //2 --- embedding_base_addr(63, 32)
  //3 --- edge_base_addr(31, 0)
  //4 --- edge_base_addr(63, 32)
  //5 --- level_base_addr(31, 0)
  //6 --- level_base_addr(63, 32)
  //7 --- root
  val controls = Module(new LookupTable(depth = 8, AXI_ADDR_WIDTH = AXI_ADDR_WIDTH))
  val start = controls.io.data(0)(0)
  val level = RegInit(0.U(32.W))
  controls.config <> io.config

  val pl_mc = Module(new multi_port_mc(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
  val Scatters = Seq.fill(16)(Module(new Scatter))
  val Gathers = Module(new Gather())
  val Applys = Module(new Apply())
  val Broadcasts = Module(new Broadcast())
  val bxbar = Module(new broadcast_xbar(64, 16, 1))

  io.PLmemory <> pl_mc.io.ddr_out
  Broadcasts.io.embedding_base_addr := Cat(controls.io.data(2), controls.io.data(1))
  Broadcasts.io.edge_base_addr := Cat(controls.io.data(4), controls.io.data(3))
  Applys.io.level_base_addr := Cat(controls.io.data(6), controls.io.data(5))
  Applys.io.level_size := 26.U
  Applys.io.level := level
  Applys.io.gather_in.tdata := Gathers.io.gather_out.tdata
  Applys.io.gather_in.tkeep := Gathers.io.gather_out.tkeep
  Applys.io.gather_in.tvalid := Gathers.io.gather_out.tvalid
  Applys.io.gather_in.tlast := Gathers.io.gather_out.tlast
  Broadcasts.io.gather_in.tdata := Gathers.io.gather_out.tdata
  Broadcasts.io.gather_in.tkeep := Gathers.io.gather_out.tkeep
  Broadcasts.io.gather_in.tvalid := Gathers.io.gather_out.tvalid
  Broadcasts.io.gather_in.tlast := Gathers.io.gather_out.tlast
  Gathers.io.gather_out.tready := Broadcasts.io.gather_in.tready & Applys.io.gather_in.tready
  Gathers.io.ddr_in <> pl_mc.io.cacheable_out
  bxbar.io.ddr_in(0) <> Broadcasts.io.xbar_out

  Broadcasts.io.ddr_r.rvalid := pl_mc.io.non_cacheable_in.rvalid
  Broadcasts.io.ddr_r.rdata := pl_mc.io.non_cacheable_in.rdata
  Broadcasts.io.ddr_r.rid := pl_mc.io.non_cacheable_in.rid
  Broadcasts.io.ddr_r.rlast := pl_mc.io.non_cacheable_in.rlast
  Broadcasts.io.ddr_r.rresp := pl_mc.io.non_cacheable_in.rresp
  pl_mc.io.non_cacheable_in.rready := Broadcasts.io.ddr_r.rready

  pl_mc.io.non_cacheable_in.araddr := Broadcasts.io.ddr_ar.araddr
  pl_mc.io.non_cacheable_in.arid := Broadcasts.io.ddr_ar.arid
  pl_mc.io.non_cacheable_in.arvalid := Broadcasts.io.ddr_ar.arvalid
  pl_mc.io.non_cacheable_in.arsize := Broadcasts.io.ddr_ar.arsize
  pl_mc.io.non_cacheable_in.arlen := Broadcasts.io.ddr_ar.arlen
  pl_mc.io.non_cacheable_in.arlock := Broadcasts.io.ddr_ar.arlock
  pl_mc.io.non_cacheable_in.arburst := Broadcasts.io.ddr_ar.arburst
  Broadcasts.io.ddr_ar.arready := pl_mc.io.non_cacheable_in.arburst

  pl_mc.io.non_cacheable_in.wvalid := Applys.io.ddr_w.wvalid
  pl_mc.io.non_cacheable_in.wlast := Applys.io.ddr_w.wlast
  pl_mc.io.non_cacheable_in.wdata := Applys.io.ddr_w.wdata
  pl_mc.io.non_cacheable_in.wstrb := Applys.io.ddr_w.wstrb
  Applys.io.ddr_w.wready := pl_mc.io.non_cacheable_in.wready

  pl_mc.io.non_cacheable_in.awvalid := Applys.io.ddr_aw.awvalid
  pl_mc.io.non_cacheable_in.awid := Applys.io.ddr_aw.awid
  pl_mc.io.non_cacheable_in.awsize := Applys.io.ddr_aw.awsize
  pl_mc.io.non_cacheable_in.awlen := Applys.io.ddr_aw.awlen
  pl_mc.io.non_cacheable_in.awburst := Applys.io.ddr_aw.awburst
  pl_mc.io.non_cacheable_in.awlock := Applys.io.ddr_aw.awlock
  pl_mc.io.non_cacheable_in.awaddr := Applys.io.ddr_aw.awaddr
  Applys.io.ddr_aw.awready := pl_mc.io.non_cacheable_in.awready

  pl_mc.io.non_cacheable_in.bready := Applys.io.ddr_b.bready
  Applys.io.ddr_b.bvalid := pl_mc.io.non_cacheable_in.bvalid
  Applys.io.ddr_b.bresp := pl_mc.io.non_cacheable_in.bresp
  Applys.io.ddr_b.bid := pl_mc.io.non_cacheable_in.bid

  //val PEs_test_finish = Wire(Vec(16, Bool()))
  //val PEs_end = Wire(Vec(16, Bool()))

  //pl_mc.io.xbar_in <> bxbar.io.ddr_out(0)
  Scatters.zipWithIndex.map{
    case (pe, i) => {
      pe.io.xbar_in <> bxbar.io.pe_out(i)
      pe.io.ddr_out <> pl_mc.io.cacheable_in(i)
     /* pe.io.start := start
      pe.io.level := level
      pe.io.root := controls.io.data(7)
      PEs_end(i) := pe.io.test_finish
      PEs_test_finish(i) := pe.io.test_finish*/
    }
  }


  /*when(PEs_end.reduce(_&_)) {
    level := 0.U
  }.elsewhen(PEs_test_finish.reduce(_&_) && Applys.io.write_finish && Broadcasts.io.read_finish){
    level := level + 1.U
  }*/
}