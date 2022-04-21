package bfs

import chisel3._
import chisel3.experimental.ChiselEnum
import nf_arm_doce._
import chisel3.util._
import utils._

//TODO: rename to cache
class WB_engine(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val ddr_aw = Decoupled(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Decoupled(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(Decoupled(new axib(AXI_ID_WIDTH, 1)))

    val xbar_in = Flipped(Decoupled(new axisdata(4)))
    val level_base_addr = Input(UInt(64.W))
    //val level_size = Input(UInt(64.W))     //2^level_size in bytes
    val level = Input(UInt(32.W))
    val end = Output(Bool())
    val flush = Input(Bool())
  })
  val buffer = Module(new URAM_cluster(256 * 1024,48))
  val region_counter = Module(new BRAM(4096, 9, "region_counter"))

  def get_level_addr(block_index: UInt, size : UInt) : UInt = {
    Cat(block_index, size.asTypeOf(UInt(6.W)))
  }

  def counter_inc(counter : UInt) : UInt = {
    Mux(counter === 63.U, 0.U, counter + 1.U)
  }

  //read region counter
  val vid = io.xbar_in.bits.tdata
  val level = io.level
  val dramaddr = (vid << 2.U).asTypeOf(UInt(AXI_ADDR_WIDTH.W))
  val block_index = dramaddr(25, 14)

  region_counter.io.clkb := clock.asBool()
  region_counter.io.clka := clock.asBool()
  region_counter.io.ena := true.B
  region_counter.io.enb := true.B
  val region_counter_addrb_0 = block_index

  //update buffer and region counter
  val region_counter_doutb_forward = Module(new axis_reg_slice(2, "region_counter_doutb_forward_reg_slice"))
  region_counter_doutb_forward.io.aclk := clock.asBool()
  region_counter_doutb_forward.io.aresetn := ~reset.asBool()
  val region_counter_doutb = Wire(UInt(9.W))
  val pipeline_1 = Module(new axis_reg_slice(4, "WB_engine_in_reg_slice"))
  val pipeline_1_out = Wire(new Bundle() {
    val addr = UInt(16.W)
    val block_index = UInt(12.W)
  })
  pipeline_1_out.addr := pipeline_1.io.m_axis.tdata(27, 12)
  pipeline_1_out.block_index := pipeline_1.io.m_axis.tdata(11, 0)
  pipeline_1.io.aclk := clock.asBool()
  pipeline_1.io.aresetn := ~reset.asBool()
  pipeline_1.io.s_axis.tvalid := io.xbar_in.valid && io.xbar_in.ready
  pipeline_1.io.s_axis.tdata := Cat(dramaddr(13, 0).asTypeOf(UInt(16.W)), block_index.asTypeOf((UInt(12.W))))
  io.xbar_in.ready := pipeline_1.io.s_axis.tready

  buffer.io.ena := true.B
  buffer.io.wea := pipeline_1.io.m_axis.tvalid.asBool() && pipeline_1.io.m_axis.tready.asBool()
  buffer.io.addra := get_level_addr(pipeline_1_out.block_index, region_counter_doutb)
  buffer.io.dina := Cat(pipeline_1_out.addr, level)
  buffer.io.clka := clock.asBool()

  val region_counter_addra_0 = pipeline_1_out.block_index
  val region_counter_wea_0 = pipeline_1.io.m_axis.tvalid.asBool() && pipeline_1.io.m_axis.tready.asBool()
  val region_counter_dina_0 = counter_inc(region_counter_doutb)

  region_counter_doutb_forward.io.s_axis.tvalid := io.xbar_in.valid && io.xbar_in.ready &&
    region_counter_addra_0 === region_counter_addrb_0 && region_counter_wea_0
  region_counter_doutb_forward.io.s_axis.tdata := region_counter_dina_0
  region_counter_doutb := Mux(region_counter_doutb_forward.io.m_axis.tvalid.asBool(),
    region_counter_doutb_forward.io.m_axis.tdata, region_counter.io.doutb)

  //write back buffer
  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val wb_level = Value(0x1.U)
    val wb_1block = Value(0x2.U)
    val check_size = Value(0x3.U)
  }
  val wb_sm = RegInit(sm.idole)
  val count = RegInit(0.U(8.W))
  val aw_buffer = Module(new BRAM_fifo(32, AXI_ADDR_WIDTH, "addr_fifo"))
  val w_buffer = Module(new BRAM_fifo(32, 64, "level_fifo"))
  val wb_block_index = RegInit(0.U(12.W))
  val flush_start = (wb_sm === sm.idole) && io.flush
  val size_b = RegInit(0.U(8.W))
  val level_base_addr_reg = RegInit(0.U(64.W))
  val wb_start = pipeline_1.io.m_axis.tvalid.asBool() && region_counter_doutb === 63.U  && wb_sm === sm.idole
  val w_buffer_reg_slice = Module(new axis_reg_slice(8, "w_buffer_reg_slice"))
  w_buffer_reg_slice.io.aclk := clock.asBool()
  w_buffer_reg_slice.io.aresetn := ~reset.asBool()
  val aw_buffer_reg_slice = Module(new axis_reg_slice(8, "aw_buffer_reg_slice"))
  aw_buffer_reg_slice.io.aclk := clock.asBool()
  aw_buffer_reg_slice.io.aresetn := ~reset.asBool()
  pipeline_1.io.m_axis.tready := wb_sm === sm.idole
  region_counter_doutb_forward.io.m_axis.tready := wb_sm === sm.idole
  level_base_addr_reg := io.level_base_addr

  when(wb_start){
    size_b := 64.U
  }.elsewhen(wb_sm === sm.check_size){
    size_b := region_counter_doutb
  }

  when(wb_start){
    wb_block_index := pipeline_1_out.block_index
  }.elsewhen(flush_start){
    wb_block_index := 0.U
  }.elsewhen(wb_sm === sm.wb_1block && wb_block_index =/= (4 * 1024 - 1).U){
    wb_block_index := wb_block_index + 1.U
  }

  //TODO: speculatively check the dirty of a block
  when(flush_start){
    wb_sm := sm.check_size
  }.elsewhen(wb_sm === sm.check_size){
    when(region_counter_doutb === 0.U){
      wb_sm := sm.wb_1block
    }.otherwise{
      wb_sm := sm.wb_level
    }
  }.elsewhen(wb_start){
    wb_sm := sm.wb_level
  }.elsewhen(wb_sm === sm.wb_level && aw_buffer_reg_slice.io.s_axis.tready.asBool() &&
    w_buffer_reg_slice.io.s_axis.tready.asBool()) {
    when(count === size_b){
      when(io.flush){
        wb_sm := sm.wb_1block
      }.otherwise{
        wb_sm := sm.idole
      }
    }.otherwise{
      wb_sm := sm.wb_level
    }
  }.elsewhen(wb_sm === sm.wb_1block) {
    when(wb_block_index =/= (4 * 1024 - 1).U){
      wb_sm := sm.check_size
    }.otherwise{
      wb_sm := sm.idole
    }
  }

  when(wb_sm === sm.idole || wb_sm === sm.wb_1block){
    count := 1.U
  }.elsewhen(wb_sm === sm.wb_level && aw_buffer_reg_slice.io.s_axis.tready.asBool() &&
    w_buffer_reg_slice.io.s_axis.tready.asBool()){
    count := count + 1.U
  }

  buffer.io.enb := true.B
  buffer.io.web := false.B
  buffer.io.addrb := Mux1H(Seq(
    (wb_start) -> get_level_addr(block_index, 0.U),
    (wb_sm === sm.check_size) -> get_level_addr(block_index, 0.U),
    (wb_sm === sm.wb_level) -> get_level_addr(wb_block_index, count)
  ))
  buffer.io.dinb := 0.U
  buffer.io.clkb := clock.asBool()

  region_counter.io.addra := Mux(wb_sm === sm.wb_level, wb_block_index, region_counter_addra_0)
  region_counter.io.wea := Mux(wb_sm === sm.wb_level, true.B, region_counter_wea_0)
  region_counter.io.dina := Mux(wb_sm === sm.wb_level, 0.U, region_counter_dina_0)
  region_counter.io.addrb := MuxCase(block_index, Array(
    (wb_sm === sm.wb_1block) -> (wb_block_index + 1.U),
    (flush_start) -> 0.U,
    (wb_sm === sm.wb_level) -> pipeline_1_out.block_index
  ))

  aw_buffer_reg_slice.io.s_axis.tdata := Cat(wb_block_index, buffer.io.doutb(13 + 32,32)).asUInt()
  aw_buffer_reg_slice.io.s_axis.tvalid := wb_sm === sm.wb_level && w_buffer_reg_slice.io.s_axis.tready.asBool()
  aw_buffer_reg_slice.io.m_axis.tready := aw_buffer.is_ready()
  aw_buffer.io.clk := clock.asBool()
  aw_buffer.io.srst := reset.asBool()
  aw_buffer.io.wr_en := aw_buffer_reg_slice.io.m_axis.tvalid//wb_sm === sm.wb_level && w_buffer.io.full === false.B
  aw_buffer.io.din := aw_buffer_reg_slice.io.m_axis.tdata + level_base_addr_reg
  val alignment_addr = aw_buffer.io.dout(63, 6)
  io.ddr_aw.bits.awaddr := Cat(alignment_addr, 0.U(6.W))
  io.ddr_aw.bits.awlock := 0.U
  io.ddr_aw.bits.awid := Cat(1.U(1.W), aw_buffer.io.data_count.asTypeOf(UInt((AXI_ID_WIDTH - 1).W)))
  io.ddr_aw.bits.awlen := 0.U
  io.ddr_aw.bits.awburst := 1.U(2.W)
  io.ddr_aw.bits.awsize := 2.U
  io.ddr_aw.valid := aw_buffer.is_valid()
  aw_buffer.io.rd_en := io.ddr_aw.ready

  w_buffer_reg_slice.io.s_axis.tdata := buffer.io.doutb(31 + 6, 0)
  w_buffer_reg_slice.io.s_axis.tvalid := wb_sm === sm.wb_level && aw_buffer_reg_slice.io.s_axis.tready.asBool()
  w_buffer_reg_slice.io.m_axis.tready := w_buffer.is_ready()
  w_buffer.io.clk := clock.asBool()
  w_buffer.io.srst := reset.asBool()
  w_buffer.io.wr_en := w_buffer_reg_slice.io.m_axis.tvalid//wb_sm === sm.wb_level && aw_buffer.io.full === false.B
  w_buffer.io.din := w_buffer_reg_slice.io.m_axis.tdata
  val alignment_offset = w_buffer.io.dout(5 + 32, 32)
  io.ddr_w.bits.wdata := w_buffer.io.dout(31, 0).asTypeOf(UInt(512.W)) << (8.U * alignment_offset)
  io.ddr_w.bits.wlast := true.B
  io.ddr_w.valid := w_buffer.is_valid()
  io.ddr_w.bits.wstrb := 0xf.U(64.W) << alignment_offset
  w_buffer.io.rd_en := io.ddr_w.ready

  io.ddr_b.ready := true.B

  io.end := wb_sm === sm.wb_1block && wb_block_index === (4 * 1024 - 1).U
}

class Apply(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle(){
    val ddr_aw = Decoupled(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Decoupled(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(Decoupled(new axib(AXI_ID_WIDTH, 1)))
    val gather_in = Flipped(Decoupled(new axisdata(4)))

    //control path
    val level = Input(UInt(32.W))
    val level_base_addr = Input(UInt(64.W))
    //val level_size = Input(UInt(64.W))     //2^level_size in bytes
    //val signal = Input(Bool())    //used in prefetch mode
    val end = Output(Bool())
    val flush = Input(Bool())
  })

  //write value array
  val vertex_update_buffer = Module(new BRAM_fifo(32, 64, "update_fifo"))
  val FIN = io.gather_in.bits.tdata(31) & io.gather_in.valid
  vertex_update_buffer.io.clk := clock.asBool()
  vertex_update_buffer.io.srst := reset.asBool()
  vertex_update_buffer.io.wr_en := io.gather_in.valid & ~FIN
  vertex_update_buffer.io.din := Cat(io.gather_in.bits.tdata, io.level)

  io.gather_in.ready := vertex_update_buffer.io.full === false.B

  val update_engine = Module(new WB_engine(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_DATA_WIDTH))
  update_engine.io.xbar_in.bits.tdata := vertex_update_buffer.io.dout(63, 32)
  update_engine.io.xbar_in.valid := vertex_update_buffer.is_valid()
  update_engine.io.level_base_addr := io.level_base_addr
  //update_engine.io.level_size := io.level_size
  update_engine.io.xbar_in.bits.tlast := true.B
  update_engine.io.xbar_in.bits.tkeep := 0xff.U
  update_engine.io.level := vertex_update_buffer.io.dout(31, 0)
  vertex_update_buffer.io.rd_en := update_engine.io.xbar_in.ready

  io.ddr_aw <> update_engine.io.ddr_aw
  io.ddr_w <> update_engine.io.ddr_w
  io.ddr_b <> update_engine.io.ddr_b
  io.end := update_engine.io.end
  update_engine.io.flush := io.flush & vertex_update_buffer.is_empty()
}

class axis_arbitrator(AXIS_DATA_WIDTH: Int = 4, NUM : Int, ELEMENT_WIDTH: Int = 4) extends Module{
  val io = IO(new Bundle() {
    val xbar_in = Flipped(Decoupled(new axisdata(AXIS_DATA_WIDTH * NUM, ELEMENT_WIDTH)))
    val ddr_out = Decoupled(new axisdata(AXIS_DATA_WIDTH, ELEMENT_WIDTH))
  })
  assert(AXIS_DATA_WIDTH == ELEMENT_WIDTH)
  val in = Module(new axis_reg_slice(AXIS_DATA_WIDTH * NUM,
    "axis_arbitrator_in_reg_slice_"+(AXIS_DATA_WIDTH * NUM).toString))
  in.io.aclk := clock.asBool()
  in.io.aresetn := ~reset.asBool()
  in.io.s_axis.connectfrom(io.xbar_in.bits)
  in.io.s_axis.tvalid := io.xbar_in.valid
  io.xbar_in.ready := in.io.s_axis.tready

  val data = in.io.m_axis.tdata
  val keep = VecInit(in.io.m_axis.tkeep(AXIS_DATA_WIDTH / ELEMENT_WIDTH * NUM - 1, 0).asBools())
  val index = RegInit(VecInit(Seq.fill(AXIS_DATA_WIDTH / ELEMENT_WIDTH * NUM)(false.B)))
  val ungrant_keep = keep.zip(index).map{
    case (a, b) => a && !b
  }
  val grant = VecInit(AMBA_ArbiterCtrl(ungrant_keep))
  val choosen_keep = grant.zip(ungrant_keep).map{
    case (a, b) => a && b
  }
  val out = Module(new axis_reg_slice(AXIS_DATA_WIDTH, "axis_arbitrator_out_reg_slice_"+AXIS_DATA_WIDTH.toString))
  out.io.aclk := clock.asBool()
  out.io.aresetn := ~reset.asBool()

  index.zip(choosen_keep).map{
    case (a, b) => {
      when(out.io.s_axis.tvalid.asBool() && out.io.s_axis.tready.asBool()){
        a := a | b
      }.elsewhen(in.io.m_axis.tready.asBool() && in.io.m_axis.tvalid.asBool()){
        a := false.B
      }
    }
  }

  in.io.m_axis.tready := !choosen_keep.reduce(_|_)

  out.io.s_axis.tvalid := choosen_keep.reduce(_|_) && in.io.m_axis.tvalid.asBool()
  out.io.s_axis.tkeep := 1.U
  out.io.s_axis.tlast := true.B
  out.io.s_axis.tdata :=
    Mux1H(Seq.tabulate(NUM)(x => (choosen_keep(x) -> data((x + 1) * AXIS_DATA_WIDTH * 8 - 1, x * AXIS_DATA_WIDTH * 8))))
  out.io.m_axis.connectto(io.ddr_out.bits, 0)
  io.ddr_out.valid := out.io.m_axis.tvalid
  out.io.m_axis.tready := io.ddr_out.ready
}

class Gather(AXI_DATA_WIDTH: Int = 64, Broadcast_Num: Int) extends Module{
  val io = IO(new Bundle() {
    val ddr_in = Flipped(Decoupled(new axisdata(AXI_DATA_WIDTH, 4)))
    val gather_out = Vec(1 + Broadcast_Num, Decoupled(new axisdata(4, 4)))

    //control path
    //val signal = Input(Bool())
  })
  val broadcaster = Module(new axis_broadcaster(AXI_DATA_WIDTH, (1 + Broadcast_Num), "gather_broadcaster"))
  broadcaster.io.s_axis.connectfrom(io.ddr_in.bits)
  broadcaster.io.s_axis.tvalid := io.ddr_in.valid
  io.ddr_in.ready := broadcaster.io.s_axis.tready
  broadcaster.io.aresetn := ~reset.asBool()
  broadcaster.io.aclk := clock.asBool()

  val v2Apply_fifo = Module(new axis_data_fifo(AXI_DATA_WIDTH, "v2A_fifo"))
  v2Apply_fifo.io.s_axis_aclk := clock.asBool()
  v2Apply_fifo.io.s_axis_aresetn := ~reset.asBool()
  v2Apply_fifo.io.s_axis.tdata := broadcaster.io.m_axis.tdata(AXI_DATA_WIDTH * 8 * (Broadcast_Num + 1) - 1, AXI_DATA_WIDTH * 8 * Broadcast_Num)
  v2Apply_fifo.io.s_axis.tvalid := broadcaster.io.m_axis.tvalid(Broadcast_Num)
  v2Apply_fifo.io.s_axis.tkeep := broadcaster.io.m_axis.tkeep(AXI_DATA_WIDTH * (Broadcast_Num + 1) - 1, AXI_DATA_WIDTH * Broadcast_Num)
  v2Apply_fifo.io.s_axis.tlast := broadcaster.io.m_axis.tlast(Broadcast_Num)

  val v2Broadcast_fifo = Seq.tabulate(Broadcast_Num)(
    i => Module(new axis_data_fifo(AXI_DATA_WIDTH / Broadcast_Num, "v2B_fifo"))
  )
  v2Broadcast_fifo.zipWithIndex.map{
    case (b, i) => {
      b.io.s_axis_aresetn := ~reset.asBool()
      b.io.s_axis_aclk := clock.asBool()
      b.io.s_axis.tdata := VecInit(broadcaster.io.m_axis.tdata(AXI_DATA_WIDTH * 8 * (i + 1) - 1, AXI_DATA_WIDTH * 8 * i).
        asTypeOf(Vec(Broadcast_Num, UInt((AXI_DATA_WIDTH * 8 / Broadcast_Num).W))).map{
        x => x((AXI_DATA_WIDTH * 8 / Broadcast_Num) / Broadcast_Num * (i + 1) - 1, (AXI_DATA_WIDTH * 8 / Broadcast_Num) / Broadcast_Num * i)
      }).asUInt()
      b.io.s_axis.tvalid := broadcaster.io.m_axis.tvalid(i) && (broadcaster.io.m_axis.tkeep(AXI_DATA_WIDTH / 4 - 1, 0).
        asTypeOf(Vec(AXI_DATA_WIDTH / 4, Bool())).zipWithIndex.map{
        case(k, x) => {
          if((x % Broadcast_Num) == i){
            k
          }else{
            false.B
          }
        }
      }.reduce(_|_))
      b.io.s_axis.tkeep := VecInit(broadcaster.io.m_axis.tkeep(AXI_DATA_WIDTH / 4 - 1, 0).asTypeOf(Vec(Broadcast_Num, UInt((AXI_DATA_WIDTH / 4 / Broadcast_Num).W))).map{
        x => x(i)
      }).asUInt()
      b.io.s_axis.tlast := broadcaster.io.m_axis.tlast(i)
    }
  }

  broadcaster.io.m_axis.tready := Cat(v2Apply_fifo.io.s_axis.tready, VecInit.tabulate(Broadcast_Num)(i => v2Broadcast_fifo(i).io.s_axis.tready).asUInt())

  val v2Apply_selecter = Module(new axis_arbitrator(4, AXI_DATA_WIDTH / 4, 4))
  v2Apply_selecter.io.xbar_in.valid := v2Apply_fifo.io.m_axis.tvalid
  v2Apply_fifo.io.m_axis.tready := v2Apply_selecter.io.xbar_in.ready
  v2Apply_fifo.io.m_axis.connectto(v2Apply_selecter.io.xbar_in.bits, 0)
  v2Apply_selecter.io.ddr_out <> io.gather_out(Broadcast_Num)

  val v2Broadcast_selecter = Seq.tabulate(Broadcast_Num)(
    i => Module(new axis_arbitrator(4, AXI_DATA_WIDTH / 4 / Broadcast_Num, 4))
  )
  v2Broadcast_selecter.zipWithIndex.map{
    case(s, i) => {
      s.io.xbar_in.valid := v2Broadcast_fifo(i).io.m_axis.tvalid
      v2Broadcast_fifo(i).io.m_axis.connectto(s.io.xbar_in.bits, 0)
      v2Broadcast_fifo(i).io.m_axis.tready := s.io.xbar_in.ready
      s.io.ddr_out <> io.gather_out(i)
    }
  }
}

//TODO: add read cache
class readEdge_engine(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3,
                      EMBEDDING : Int = 14, Broadcast_Num : Int, Broadcast_index : Int) extends Module{
  val io = IO(new Bundle() {
    val in = Flipped(Decoupled(new axir(AXI_DATA_WIDTH, AXI_ID_WIDTH)))
    val out = Decoupled(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    //control signal
    val edge_base_addr = Input(UInt(64.W))
    val free_ptr = Input(UInt((AXI_ID_WIDTH-1).W))
    val read_edge_num = Output(UInt(32.W))
    val read_edge_fifo_empty = Output(Bool())
    val inflight_vtxs = Input(UInt(64.W))
  })

  def get_edge_array_index(mdata : UInt) : UInt = {
    mdata(31, 0)
  }

  def get_edge_count(mdata : UInt) : UInt = {
    mdata(63, 32)
  }

  def get_flag(id : UInt) : Bool = {
    id(AXI_ID_WIDTH - 1).asBool()
  }

  def get_ptr(id : UInt) : UInt = {
    id(AXI_ID_WIDTH - 1 - 1, 0)
  }

  //read edge array
  val edge_read_buffer = Module(new BRAM_fifo(32, 64, "meta_fifo"))
  val transaction_start = RegInit(false.B)
  when(io.in.valid && io.in.ready && !io.in.bits.rlast.asBool()) {
    transaction_start := true.B
  }.elsewhen(io.in.valid && io.in.ready && io.in.bits.rlast.asBool()){
    transaction_start := false.B
  }
  edge_read_buffer.io.clk := clock.asBool()
  edge_read_buffer.io.srst := reset.asBool()
  edge_read_buffer.io.din := io.in.bits.rdata(63, 0)
  edge_read_buffer.io.wr_en := !get_flag(io.in.bits.rid) & io.in.valid &
    (get_edge_count(io.in.bits.rdata) > EMBEDDING.asUInt()) & (transaction_start === false.B)
  io.in.ready := edge_read_buffer.io.full === false.B

  val remainning_edges = (get_edge_count(edge_read_buffer.io.dout) - EMBEDDING.asUInt())
  object cache_sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val next_page  = Value(0x1.U) // i "load"  -> 000_0011
  }
  val cache_status = RegInit(cache_sm.idole)
  val counter = RegInit(0.U(32.W))
  val next_counter = Wire(UInt(32.W))
  val araddr = RegInit(0.U(AXI_ADDR_WIDTH.W))
  val max_vertex_read_per_transaction = 64.U
  next_counter := counter - max_vertex_read_per_transaction
  when(remainning_edges.asUInt() > max_vertex_read_per_transaction && io.out.ready && io.out.valid && cache_status === cache_sm.idole){
    cache_status := cache_sm.next_page
    counter := remainning_edges - max_vertex_read_per_transaction
    araddr := io.edge_base_addr + (get_edge_array_index(edge_read_buffer.io.dout) << 2).asUInt() + (max_vertex_read_per_transaction << 2).asUInt()
  }.elsewhen(io.out.ready && io.out.valid && cache_status === cache_sm.next_page){
    when(counter <= max_vertex_read_per_transaction){
      cache_status := cache_sm.idole
      counter := 0.U
      araddr := 0.U
    }.otherwise{
      counter := next_counter
      araddr := araddr + (max_vertex_read_per_transaction << 2).asUInt()
    }
  }

  val num_vertex = MuxCase(max_vertex_read_per_transaction, Array(
    (cache_status === cache_sm.idole && remainning_edges <= max_vertex_read_per_transaction) -> (remainning_edges),
    (cache_status === cache_sm.next_page && counter <= max_vertex_read_per_transaction) -> counter
  ))
  var arlen = Mux(((num_vertex >> log2Ceil(AXI_DATA_WIDTH/4)) << log2Ceil(AXI_DATA_WIDTH/4)).asUInt() < num_vertex,
    (num_vertex >> log2Ceil(AXI_DATA_WIDTH/4)),
    (num_vertex >> log2Ceil(AXI_DATA_WIDTH/4)).asUInt() - 1.U)
  io.out.bits.araddr := Mux(cache_status === cache_sm.idole,
    io.edge_base_addr + (get_edge_array_index(edge_read_buffer.io.dout) << 2),
    araddr)
  io.out.valid := Mux(cache_status === cache_sm.idole, edge_read_buffer.is_valid(), true.B) && io.inflight_vtxs < 32.U
  io.out.bits.arlen := arlen.asUInt()
  io.out.bits.arburst := 1.U(2.W)
  io.out.bits.arlock := 0.U
  io.out.bits.arsize := MuxCase(log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W), Seq.tabulate(log2Ceil(AXI_DATA_WIDTH/4))(
    x => ((num_vertex <= Seq.tabulate(log2Ceil(AXI_DATA_WIDTH/4))(i => 1 << i)(x).asUInt()) -> (x + 2).asUInt())
  ))
  io.out.bits.arid := Cat(1.U(1.W), io.free_ptr.asTypeOf(UInt((AXI_ID_WIDTH- 1).W)))

  edge_read_buffer.io.rd_en := io.out.ready & cache_status === cache_sm.idole && io.inflight_vtxs < 32.U
  io.read_edge_num := num_vertex
  io.read_edge_fifo_empty := edge_read_buffer.is_empty() && cache_status === cache_sm.idole
}

class Broadcast(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3,
                EMBEDDING : Int = 14, Broadcast_Num : Int, Broadcast_index : Int,
                FPGA_Num: Int) extends Module{
  val io = IO(new Bundle() {
    val ddr_ar = Decoupled(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val ddr_r = Flipped(Decoupled(new axir(AXI_DATA_WIDTH, AXI_ID_WIDTH)))
    val gather_in = Flipped(Decoupled(new axisdata(4, 4)))
    val xbar_out = Decoupled(new axisdata(AXI_DATA_WIDTH, 4))

    //control path
    val embedding_base_addr = Input(UInt(64.W))
    val edge_base_addr = Input(UInt(64.W))
    val signal = Input(Bool())
    val traveled_edges = Output(UInt(64.W))       //every super step
    val start = Input(Bool())
    val root = Input(UInt(32.W))
    val issue_sync = Output(Bool())
    val recv_sync = Input(UInt(Broadcast_Num.W))
  })
  assert(AXI_DATA_WIDTH >= 8)
  assert(AXI_ID_WIDTH > log2Ceil(32))

  def vid2addr(vid: UInt) : UInt = {
    io.embedding_base_addr + (vid << log2Ceil(4 * EMBEDDING + 8))
  }

  //upward
  object upward_sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val exe  = Value(0x1.U) // i "load"  -> 000_0011
    val fin   = Value(0x2.U) // i "imm"   -> 001_0011
    val output_fin = Value(0x3.U)
    val sync = Value(0x4.U)
  }
  val upward_status = RegInit(upward_sm.idole)
  val inflight_vtxs = RegInit(0.U(64.W))
  val traveled_edges_reg = RegInit(0.U(64.W))
  io.traveled_edges := traveled_edges_reg

  //read offset and edge embedding array
  val vertex_read_buffer = Module(new BRAM_fifo(32, 32, "vid_fifo"))
  vertex_read_buffer.io.clk := clock.asBool()
  vertex_read_buffer.io.srst := reset.asBool()
  vertex_read_buffer.io.wr_en := io.gather_in.valid
  vertex_read_buffer.io.din := io.gather_in.bits.tdata
  io.gather_in.ready := vertex_read_buffer.io.full === false.B
  val vertex_read_id = Wire(UInt(AXI_ID_WIDTH.W))
  vertex_read_id := Cat(0.U(1.W), vertex_read_buffer.io.data_count.asTypeOf(UInt((AXI_ID_WIDTH- 1).W)))

  val edge_cache = Module(new readEdge_engine(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, EMBEDDING,
    Broadcast_Num, Broadcast_index))
  edge_cache.io.edge_base_addr := io.edge_base_addr
  edge_cache.io.inflight_vtxs := inflight_vtxs
  val r_demux = Module(new axis_broadcaster(AXI_DATA_WIDTH, 2, "r_demux", AXI_ID_WIDTH))
  val r_demux_out = Wire(Vec(2, Decoupled(new axir(AXI_DATA_WIDTH, AXI_ID_WIDTH))))
  r_demux.io.s_axis.tvalid := io.ddr_r.valid
  r_demux.io.s_axis.tdata := io.ddr_r.bits.rdata
  r_demux.io.s_axis.tlast := io.ddr_r.bits.rlast
  r_demux.io.s_axis.disable_tkeep()
  r_demux.io.s_axis.tid := io.ddr_r.bits.rid
  r_demux.io.aclk := clock.asBool()
  r_demux.io.aresetn := ~reset.asBool()
  io.ddr_r.ready := r_demux.io.s_axis.tready
  r_demux_out.zipWithIndex.map{
    case (r, i) => {
      r.valid := r_demux.io.m_axis.tvalid(i)
      r.bits.rid := r_demux.io.m_axis.tid(AXI_ID_WIDTH * (i + 1) - 1, AXI_ID_WIDTH * i)
      r.bits.rlast := r_demux.io.m_axis.tlast(i)
      r.bits.rdata := r_demux.io.m_axis.tdata(8 * AXI_DATA_WIDTH * (i + 1) - 1, 8 * AXI_DATA_WIDTH * i)
    }
  }
  r_demux.io.m_axis.tready := VecInit.tabulate(2)(i => r_demux_out(i).ready).asUInt()
  edge_cache.io.in <> r_demux_out(1)

  val arbi = Module(new AMBA_Arbiter(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1), 2))
  io.ddr_ar <> arbi.io.out
  arbi.io.in(1).bits.araddr :=  vid2addr(vertex_read_buffer.io.dout)
  arbi.io.in(1).valid :=  vertex_read_buffer.is_valid() && vertex_read_buffer.io.dout(31) === 0.U && inflight_vtxs < 32.U
  arbi.io.in(1).bits.arlen := ((4 * EMBEDDING + 8) / AXI_DATA_WIDTH - 1).asUInt()
  arbi.io.in(1).bits.arburst := 1.U(2.W)
  arbi.io.in(1).bits.arlock := 0.U
  arbi.io.in(1).bits.arsize :=  log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W)
  arbi.io.in(1).bits.arid :=  vertex_read_id
  vertex_read_buffer.io.rd_en := (arbi.io.in(1).ready && inflight_vtxs < 32.U) | upward_status === upward_sm.output_fin
  arbi.io.in(0) <> edge_cache.io.out

  //front end of down ward
  val num_regfile = Module(new regFile(32, 32))
  num_regfile.io.writeFlag := arbi.io.in(0).ready & arbi.io.in(0).valid
  num_regfile.io.wptr := num_regfile.io.wcount
  num_regfile.io.dataIn := edge_cache.io.read_edge_num
  num_regfile.io.rptr := edge_cache.get_ptr(r_demux_out(0).bits.rid)
  edge_cache.io.free_ptr := num_regfile.io.wcount

  //back end of down ward
  val KEEP_WIDTH = AXI_DATA_WIDTH / 4
  object sm extends ChiselEnum {
    //val idole = Value(0x0.U)
    val firstBurst  = Value(0x0.U) // i "load"  -> 000_0011
    val remainingBurst_embedding   = Value(0x1.U) // i "imm"   -> 001_0011
    val remainingBurst_edge   = Value(0x2.U)
  }
  val status = RegInit(sm.firstBurst)
  val num = RegInit(0.U(32.W))
  when(status === sm.firstBurst && r_demux_out(0).valid.asBool() && r_demux_out(0).ready.asBool()){
    when(r_demux_out(0).bits.rlast.asBool()){
      status := sm.firstBurst
    }.elsewhen(edge_cache.get_flag(r_demux_out(0).bits.rid)){
      status := sm.remainingBurst_edge
    }.otherwise{
      status := sm.remainingBurst_embedding
    }
  }.elsewhen((status === sm.remainingBurst_edge  || status === sm.remainingBurst_embedding)
            && r_demux_out(0).valid.asBool() && r_demux_out(0).ready.asBool() && r_demux_out(0).bits.rlast.asBool()){
      status := sm.firstBurst
  }

  when(status === sm.firstBurst
      && r_demux_out(0).valid.asBool() && r_demux_out(0).ready.asBool() && !r_demux_out(0).bits.rlast.asBool()){
    when(edge_cache.get_flag(r_demux_out(0).bits.rid)){
      num := num_regfile.io.dataOut
    }.otherwise{
      num := edge_cache.get_edge_count(r_demux_out(0).bits.rdata)
    }
  }.elsewhen(status === sm.remainingBurst_edge && r_demux_out(0).valid.asBool() && r_demux_out(0).ready.asBool()){
    when(r_demux_out(0).bits.rlast.asBool()){
      num := 0.U
    }.otherwise{
      num := num - KEEP_WIDTH.U
    }
  }.elsewhen(status === sm.remainingBurst_embedding && r_demux_out(0).valid.asBool() && r_demux_out(0).ready.asBool()){
    when(r_demux_out(0).bits.rlast.asBool()){
      num := 0.U
    }.otherwise{
      num := Mux(num > KEEP_WIDTH.U, num - KEEP_WIDTH.U, 0.U)
    }
  }

  val keep = Wire(Vec(KEEP_WIDTH, Bool()))
  keep.zipWithIndex.map {
    case(k, i) => k := MuxCase(false.B, Array(
      (status === sm.firstBurst && edge_cache.get_flag(r_demux_out(0).bits.rid)) -> (num_regfile.io.dataOut > i.U),
      (status === sm.remainingBurst_edge) -> (num > i.U),
      (status === sm.firstBurst && !edge_cache.get_flag(r_demux_out(0).bits.rid)) ->
        ((i.U > 1.U) && ((edge_cache.get_edge_count(r_demux_out(0).bits.rdata) + 2.U) > i.U)),
      (status === sm.remainingBurst_embedding) -> (num > (i.U + KEEP_WIDTH.U - 2.U))
    ))
  }

  val vertex_out_fifo = Module(new BRAM_fifo(32, AXI_DATA_WIDTH * 8 + KEEP_WIDTH, "edge_fifo"))
  vertex_out_fifo.io.srst := reset.asBool()
  vertex_out_fifo.io.clk := clock.asBool()
  io.xbar_out.valid := vertex_out_fifo.is_valid()
  io.xbar_out.bits.tkeep := vertex_out_fifo.io.dout(KEEP_WIDTH - 1, 0)
  io.xbar_out.bits.tdata := vertex_out_fifo.io.dout(AXI_DATA_WIDTH * 8 + KEEP_WIDTH - 1, KEEP_WIDTH)
  io.xbar_out.bits.tlast := true.B
  vertex_out_fifo.io.rd_en := io.xbar_out.ready
  vertex_out_fifo.io.wr_en := r_demux_out(0).valid | upward_status === upward_sm.output_fin | io.start
  vertex_out_fifo.io.din := MuxCase(Cat(r_demux_out(0).bits.rdata, keep.asUInt()), Array(
    io.start -> Cat(io.root, 0x1.U(KEEP_WIDTH.W)),
    (upward_status === upward_sm.output_fin) -> Cat("x80000000".asUInt((AXI_DATA_WIDTH * 8).W), 0x1.U(KEEP_WIDTH.W))
  ))
  r_demux_out(0).ready := vertex_out_fifo.io.full === false.B

  //control path
  val syncRecv = RegInit(VecInit(Seq.fill(Broadcast_Num)(false.B)))
  syncRecv.zipWithIndex.map{
    case(f, i) => {
      when(io.signal){
        f := false.B
      }.elsewhen(io.recv_sync(i)){
        f := true.B
      }
    }
  }
  io.issue_sync := upward_status === upward_sm.sync
  when(io.signal && (upward_status === upward_sm.idole)){
    upward_status := upward_sm.exe
  }.elsewhen(upward_status === upward_sm.exe && (vertex_read_buffer.test_FIN() || syncRecv.reduce(_|_))){
    upward_status := upward_sm.fin
  }.elsewhen(upward_status === upward_sm.fin && inflight_vtxs === 0.U && vertex_out_fifo.is_empty()
            && edge_cache.io.read_edge_fifo_empty === true.B){
    upward_status := upward_sm.sync
  }.elsewhen(upward_status === upward_sm.sync && syncRecv.reduce(_&_)){
    when(Broadcast_index.U === 0.U ){
      upward_status := upward_sm.output_fin
    }.otherwise{
      upward_status := upward_sm.idole
    }
  }.elsewhen(upward_status === upward_sm.output_fin){
    upward_status := upward_sm.idole
  }

  when(io.signal){
    traveled_edges_reg := 0.U
  }.elsewhen(r_demux_out(0).valid.asBool() && r_demux_out(0).ready.asBool() && !edge_cache.get_flag(r_demux_out(0).bits.rid)
    && status === sm.firstBurst){
    traveled_edges_reg := traveled_edges_reg + edge_cache.get_edge_count(r_demux_out(0).bits.rdata)
  }

  when(io.ddr_ar.valid && io.ddr_ar.ready
    && io.ddr_r.valid.asBool() && io.ddr_r.ready.asBool() && io.ddr_r.bits.rlast.asBool()){
    inflight_vtxs := inflight_vtxs
  }.elsewhen(io.ddr_ar.valid && io.ddr_ar.ready){
    inflight_vtxs := inflight_vtxs + 1.U
  }.elsewhen(io.ddr_r.valid.asBool() && io.ddr_r.ready.asBool() && io.ddr_r.bits.rlast.asBool()){
    inflight_vtxs := inflight_vtxs - 1.U
  }
}

/*
* PE ID is global addressable
* MC ID is local addressable
* */
class broadcast_xbar(AXIS_DATA_WIDTH: Int, SLAVE_NUM: Int, MASTER_NUM: Int) extends Module {
  val io = IO(new Bundle {
    val ddr_in = Vec(MASTER_NUM, Flipped(Decoupled(new axisdata(AXIS_DATA_WIDTH, 4))))
    val pe_out = Vec(SLAVE_NUM, Decoupled(new axisdata(AXIS_DATA_WIDTH * MASTER_NUM, 4)))
  })
  //ddr_in to pe_out direction
  val xbar = Module(new axis_broadcaster(AXIS_DATA_WIDTH * MASTER_NUM, SLAVE_NUM,
    "axis_broadcaster_"+(AXIS_DATA_WIDTH * MASTER_NUM).toString))
  if(MASTER_NUM == 1){
    xbar.io.s_axis.connectfrom(io.ddr_in(0).bits)
    xbar.io.s_axis.tvalid := io.ddr_in(0).valid
    io.ddr_in(0).ready := xbar.io.s_axis.tready
  }else{
    val combiner = Module(new axis_combiner(AXIS_DATA_WIDTH, MASTER_NUM, "axis_combiner_level0"))
    combiner.io.aclk := clock.asBool()
    combiner.io.aresetn := ~reset.asBool()
    combiner.io.s_axis.tdata := VecInit.tabulate(MASTER_NUM){i => io.ddr_in(i).bits.tdata}.asUInt()
    combiner.io.s_axis.tkeep := VecInit.tabulate(MASTER_NUM){i => VecInit(io.ddr_in(i).bits.tkeep.asBools().map{x => x &
      io.ddr_in(i).valid})}.asUInt()
    combiner.io.s_axis.tlast := VecInit.tabulate(MASTER_NUM){i => io.ddr_in(i).bits.tlast}.asUInt()
    combiner.io.s_axis.tvalid := VecInit(Seq.fill(MASTER_NUM)(io.ddr_in.map{i => i.valid}.reduce(_|_))).asUInt()
    io.ddr_in.zipWithIndex.map{
      case(in, i) => {in.ready := combiner.io.s_axis.tready(i)}
    }

    val buffer0 = Module(new axis_data_fifo(AXIS_DATA_WIDTH * MASTER_NUM, "Remote_xbar_buffer0"))
    buffer0.io.s_axis_aclk := clock.asBool()
    buffer0.io.s_axis_aresetn := ~reset.asBool()
    buffer0.io.s_axis <> combiner.io.m_axis
    xbar.io.s_axis.tdata := buffer0.io.m_axis.tdata
    xbar.io.s_axis.tvalid := buffer0.io.m_axis.tvalid
    xbar.io.s_axis.tkeep := VecInit(buffer0.io.m_axis.tkeep.asBools().map{x => x & buffer0.io.m_axis.tvalid}).asUInt()
    buffer0.io.m_axis.tready := xbar.io.s_axis.tready
  }

  xbar.io.aclk := clock.asBool()
  xbar.io.aresetn := ~reset.asBool()
  io.pe_out.zipWithIndex.map{
    case(pe, i) => {
      xbar.io.m_axis.connectto(pe.bits, i)
      pe.valid := xbar.io.m_axis.tvalid(i)
    }
  }
  xbar.io.m_axis.tready := (VecInit.tabulate(SLAVE_NUM)(
    i => io.pe_out(i).ready
  )).asUInt()
}

/*
* end: there is no more vertexes for the level
* 1M vertex for each scatter
* */
class Scatter(AXIS_DATA_WIDTH: Int = 4, SID: Int, AXI_DATA_WIDTH: Int,
              FPGA_Num: Int, Apply_num : Int) extends Module {
  val io = IO(new Bundle() {
    val xbar_in = Flipped(Decoupled(new axisdata(AXI_DATA_WIDTH, 4)))
    val ddr_out = Decoupled(new axisdata(AXIS_DATA_WIDTH, 4))

    //control path
    val end = Output(Bool())
    val local_fpga_id = Input(UInt(log2Ceil(FPGA_Num).W))
  })
  val vertex_num = AXI_DATA_WIDTH / 4
  val bitmap = Module(new BRAM(116 * 1024 * 16 / Apply_num, 9, "bitmap_0"))
  val arbi = Module(new axis_arbitrator(4, vertex_num, 4))

  def vid_to_sid(vid: UInt, sid: UInt) : Bool = {
    if(FPGA_Num == 1) {
      vid(log2Ceil(Apply_num) - 1, 0) === sid
    }else{
      vid(log2Ceil(Apply_num) + log2Ceil(FPGA_Num) - 1, log2Ceil(FPGA_Num)) === sid &&
        vid(log2Ceil(FPGA_Num) - 1, 0) === io.local_fpga_id
    }
  }

  def vid2bitmap_addr(vid: UInt) : UInt = {
    Mux(vid(30, log2Ceil(Apply_num) + log2Ceil(FPGA_Num)) > (8 * 116 * 1024 * 16 / Apply_num - 1).U(27.W),
      vid(30, log2Ceil(Apply_num) + log2Ceil(FPGA_Num)) - (8 * 116 * 1024 * 16 / Apply_num - 1).U(27.W),
      vid(30, log2Ceil(Apply_num) + log2Ceil(FPGA_Num) + 3).asTypeOf(UInt(27.W)))
  }

  def vid2bitmap_offset(vid: UInt) : UInt = {
    Mux(vid(30, log2Ceil(Apply_num) + log2Ceil(FPGA_Num)) > (8 * 116 * 1024 * 16 / Apply_num - 1).U(27.W),
      8.U(4.W),
      vid(log2Ceil(Apply_num) + log2Ceil(FPGA_Num) + 2, log2Ceil(Apply_num) + log2Ceil(FPGA_Num)).asTypeOf(UInt(4.W)))
  }

  val scatter_in = Module(new axis_reg_slice(128, "Scatter_in_reg_slice"))
  scatter_in.io.aclk := clock.asBool()
  scatter_in.io.aresetn := ~reset.asBool()
  scatter_in.io.s_axis.connectfrom(io.xbar_in.bits)
  scatter_in.io.s_axis.tvalid := io.xbar_in.valid
  io.xbar_in.ready := scatter_in.io.s_axis.tready

  val filtered_keep = Wire(Vec(vertex_num, Bool()))
  filtered_keep.zipWithIndex.map{
    case(k, i) => {
      k := Mux(scatter_in.io.m_axis.tkeep(i),
        Mux(scatter_in.io.m_axis.tdata(i*32 + 31), true.B, vid_to_sid(scatter_in.io.m_axis.tdata(i*32+31, i*32), SID.asUInt())),
        false.B)
    }
  }

  val vertex_in_fifo = Module(new axis_data_fifo(128, "vid_32_fifo"))
  vertex_in_fifo.io.s_axis_aclk := clock.asBool()
  vertex_in_fifo.io.s_axis_aresetn := !reset.asBool()
  vertex_in_fifo.io.s_axis.tdata := scatter_in.io.m_axis.tdata
  vertex_in_fifo.io.s_axis.tlast := scatter_in.io.m_axis.tlast
  vertex_in_fifo.io.s_axis.tvalid := scatter_in.io.m_axis.tvalid
  vertex_in_fifo.io.s_axis.tkeep := filtered_keep.asUInt()
  scatter_in.io.m_axis.tready := vertex_in_fifo.io.s_axis.tready

  vertex_in_fifo.io.m_axis.connectto(arbi.io.xbar_in.bits, 0)
  arbi.io.xbar_in.valid := vertex_in_fifo.io.m_axis.tvalid
  vertex_in_fifo.io.m_axis.tready := arbi.io.xbar_in.ready

  val vertex_out_fifo = Module(new BRAM_fifo(32, 32, "vid_fifo"))
  vertex_out_fifo.io.clk := clock.asBool()
  vertex_out_fifo.io.srst := reset.asBool()

  //read or write bitmap ehnr this is not FIN
  val bitmap_arvalid = arbi.io.ddr_out.valid
  val halt = vertex_out_fifo.io.full === true.B
  val bitmap_wait = Module(new pipeline(UInt(32.W)))
  bitmap_wait.io.din.valid := bitmap_arvalid
  bitmap_wait.io.din.bits := arbi.io.ddr_out.bits.tdata
  bitmap_wait.io.dout.ready := !halt
  val bitmap_doutb = Wire(UInt(9.W))
  val bitmap_write_addr = Module(new pipeline(UInt(32.W)))
  bitmap_write_addr.io.din.valid := bitmap_wait.io.dout.valid &&
    (bitmap_doutb(vid2bitmap_offset(bitmap_wait.io.dout.bits)) =/= 1.U(1.W) | bitmap_wait.io.dout.bits(31) === 1.U(1.W))
  bitmap_write_addr.io.din.bits := bitmap_wait.io.dout.bits
  bitmap_write_addr.io.dout.ready := !halt
  val bitmap_write_data = Module(new pipeline(UInt(9.W)))
  bitmap_write_data.io.din.valid := (bitmap_wait.io.dout.valid &&
    (bitmap_doutb(vid2bitmap_offset(bitmap_wait.io.dout.bits)) =/= 1.U(1.W) | bitmap_wait.io.dout.bits(31) === 1.U(1.W)))
  bitmap_write_data.io.din.bits := bitmap_doutb | (1.U(9.W) << vid2bitmap_offset(bitmap_wait.io.dout.bits)).asUInt()
  bitmap_write_data.io.dout.ready := !halt
  val bitmap_write_data_forward = Module(new pipeline(UInt(9.W)))
  bitmap_write_data_forward.io.din.valid := bitmap_write_addr.io.dout.valid && bitmap_arvalid &&
    (vid2bitmap_addr(arbi.io.ddr_out.bits.tdata) === vid2bitmap_addr(bitmap_write_addr.io.dout.bits))
  bitmap_write_data_forward.io.din.bits := bitmap_write_data.io.dout.bits
  bitmap_write_data_forward.io.dout.ready := !halt
  bitmap_doutb := MuxCase(bitmap.io.doutb, Array(
    (bitmap_write_addr.io.dout.valid && bitmap_wait.io.dout.valid &&
      vid2bitmap_addr(bitmap_wait.io.dout.bits) === vid2bitmap_addr(bitmap_write_addr.io.dout.bits)) -> bitmap_write_data.io.dout.bits,
    bitmap_write_data_forward.io.dout.valid -> bitmap_write_data_forward.io.dout.bits
  ))

  arbi.io.ddr_out.ready := !halt
  bitmap.io.enb :=  true.B//bitmap_arvalid
  bitmap.io.addrb := vid2bitmap_addr(arbi.io.ddr_out.bits.tdata)
  bitmap.io.clkb := clock.asBool()
  vertex_out_fifo.io.wr_en := bitmap_write_addr.io.dout.valid
  vertex_out_fifo.io.din := bitmap_write_addr.io.dout.bits
  bitmap.io.ena := true.B
  bitmap.io.wea := bitmap_write_addr.io.dout.valid & bitmap_write_addr.io.dout.bits(31) === 0.U(1.W)
  bitmap.io.clka := clock.asBool()
  bitmap.io.dina := bitmap_write_data.io.dout.bits
  bitmap.io.addra := vid2bitmap_addr(bitmap_write_addr.io.dout.bits)
  vertex_out_fifo.io.rd_en := io.ddr_out.ready //| vertex_out_fifo.test_FIN()
  io.ddr_out.valid := vertex_out_fifo.is_valid() & vertex_out_fifo.io.dout(31) === 0.U
  io.ddr_out.bits.tkeep := true.B
  io.ddr_out.bits.tlast := true.B
  io.ddr_out.bits.tdata := vertex_out_fifo.io.dout

  //control path
  io.end := vertex_out_fifo.test_FIN()

  val ready_counter = RegInit(0.U(32.W))
  dontTouch(ready_counter)
  when(io.xbar_in.ready === false.B && io.xbar_in.valid === true.B){
    ready_counter := ready_counter + 1.U
  }
}

//TODO: Try to use fifoIO
class multi_channel_fifo(AXI_DATA_WIDTH: Int = 64, size : Int = 16, val Scatter_num : Int) extends Module{
  val io = IO(new Bundle() {
    val in = Vec(Scatter_num, Flipped(Decoupled(new axisdata(4, 4))))
    val out = new Bundle() {
      val full = Output(Bool())
      val din = Input(UInt((8 * AXI_DATA_WIDTH).W))
      val wr_en = Input(Bool())
      val dout = Output(UInt((8 * AXI_DATA_WIDTH).W))
      val rd_en = Input(Bool())
      val data_count = Output(UInt((log2Ceil(size * AXI_DATA_WIDTH / 4) + 1).W))
      val valid = Output(Bool())
    }
    val flush = Input(Bool())
    val is_current_tier = Input(Bool())
  })

  assert(Scatter_num <= 16)

  def is_empty() : Bool = {
    io.out.data_count === 0.U
  }

  def is_valid() : Bool = {
    io.out.valid
  }

  val collector_fifos = Seq.tabulate(16)(
    i => Module(new BRAM_fifo(16, 32, ("collector_fifo_0")))
  )
  val fifos_ready = Seq.tabulate(16)(i => collector_fifos(i).io.full === false.B).reduce(_&_)
  val counter = RegInit(0.U(log2Ceil(16).W))
  val collector_data = Wire(Vec(16, UInt(32.W)))
  val in_pipeline = Seq.fill(Scatter_num)(
    Module(new axis_reg_slice(4, "multi_channel_fifo_reg_slice"))
  )
  in_pipeline.zipWithIndex.map{
    case(p, i) => {
      p.io.aclk := clock.asBool()
      p.io.aresetn := ~reset.asBool()
      p.io.s_axis.connectfrom(io.in(i).bits)
      p.io.s_axis.tvalid := io.in(i).valid
      io.in(i).ready := p.io.s_axis.tready
      p.io.m_axis.tready := Mux(io.is_current_tier, false.B, fifos_ready)
    }
  }
  val steps = (Seq.tabulate(Scatter_num)(
    i => {
      Seq.tabulate(i + 1)(x => (in_pipeline(x).io.m_axis.tvalid.asBool() && fifos_ready).asTypeOf(UInt(5.W))).reduce(_+_)
    }
  ))

  def indexAdd(index : UInt, step : UInt) : UInt = {
    Mux(index + step >= 16.U, index + step - 16.U, index + step)
  }
  def indexSub(index1 : UInt, index2 : UInt) : UInt = {
    Mux(index1 <= index2, index2 - index1, 16.U - index1 + index2)
  }

  when(steps(Scatter_num - 1) =/= 0.U){
    counter := indexAdd(counter, steps(Scatter_num - 1))
  }.elsewhen(io.flush){
    counter := 0.U
  }

  collector_fifos.zipWithIndex.map{
    case (f, i) => {
      val fifo_in_data = MuxCase(0.U, Seq.tabulate(Scatter_num)(
        x => ((indexSub(counter, i.U) + 1.U) === steps(x)) -> in_pipeline(x).io.m_axis.tdata)
      )
      val fifo_in_valid = MuxCase(0.U, Seq.tabulate(Scatter_num)(
        x => ((indexSub(counter, i.U) + 1.U) === steps(x)) -> in_pipeline(x).io.m_axis.tvalid)
      )
      f.io.din := Mux(io.is_current_tier, io.out.din(32 * (i + 1) - 1, 32 * i), fifo_in_data)
      f.io.wr_en := Mux(io.is_current_tier, io.out.wr_en ,fifo_in_valid)
      collector_data(i) := f.io.dout
      f.io.clk := clock.asBool()
      f.io.srst := reset.asBool()
      f.io.rd_en := io.out.rd_en
    }
  }

  io.out.valid := Seq.tabulate(16)(i => collector_fifos(i).is_valid()).reduce(_|_)
  io.out.dout := collector_data.asUInt()
  io.out.data_count := collector_fifos.map{i => i.io.data_count.asTypeOf(UInt((log2Ceil(size * AXI_DATA_WIDTH / 4) + 1).W))}.reduce(_+_)
  io.out.full := ~fifos_ready

  val ready_counter = RegInit(0.U(32.W))
  dontTouch(ready_counter)
  when(fifos_ready === false.B && in_pipeline.map{x => x.io.m_axis.tvalid.asBool()}.reduce(_|_)){
    ready_counter := ready_counter + 1.U
  }
}

/*
* mc send FIN when no more data in current tier FIFO
* FIN: vid[31] = 1
* */
class multi_port_mc(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int, AXI_SIZE_WIDTH: Int = 3,
                    Scatter_num : Int) extends Module {
  val io = IO(new Bundle() {
    val cacheable_out = Decoupled(new axisdata(AXI_DATA_WIDTH, 4))
    val cacheable_in = Vec(Scatter_num, Flipped(Decoupled(new axisdata(4, 4))))
    val non_cacheable_in = new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)
    val ddr_out = Vec(2, Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))

    //control path
    val tiers_base_addr = Vec(2, Input(UInt(AXI_ADDR_WIDTH.W)))
    val unvisited_size = Output(UInt(32.W))
    val start = Input(Bool())
    val signal = Input(Bool())
    val end = Input(Bool())
    val signal_ack = Output(Bool())
  })

  val tier_fifo = Seq.tabulate(2)(i => Module(new multi_channel_fifo(AXI_DATA_WIDTH, 16, Scatter_num)))
  val tier_counter = RegInit(VecInit(Seq.fill(2)(0.U(32.W))))

  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val next_tier_is_0  = Value(0x1.U) // i "load"  -> 000_0011
    val next_tier_is_1   = Value(0x2.U) // i "imm"   -> 001_0011
    val writeback = Value(0x3.U)
    val readback = Value(0x4.U)
    //val start = Value(0x7.U)
    val sync_send_0 = Value(0x5.U)
    val sync_send_1 = Value(0x6.U)
    val signal_wait_0 = Value(0x7.U)
    val signal_wait_1 = Value(0x8.U)
    val writebackdata = Value(0x9.U)
    val readbackdata = Value(0x10.U)
    val wait_tier0_wb = Value(0x11.U)
    val wait_tier1_wb = Value(0x12.U)
  }
  val status = RegInit(sm.idole)
  val tier_status = RegInit(VecInit(Seq.fill(2)(sm.idole)))

  when(io.start && status === sm.idole){
    status := sm.next_tier_is_1
  }.elsewhen(status === sm.next_tier_is_1 && tier_counter(0) === 0.U){
    status := sm.sync_send_0
  }.elsewhen(status === sm.sync_send_0 && io.cacheable_out.valid && io.cacheable_out.ready){
    status := sm.signal_wait_0
  }.elsewhen(io.signal && status === sm.signal_wait_0){
    when(tier_status(1) =/= sm.idole) {
      status := sm.wait_tier1_wb
    }.otherwise{
      status := sm.next_tier_is_0
    }
  }.elsewhen(status === sm.next_tier_is_0 && tier_counter(1) === 0.U){
    status := sm.sync_send_1
  }.elsewhen(status === sm.sync_send_1 && io.cacheable_out.valid && io.cacheable_out.ready){
    status := sm.signal_wait_1
  }.elsewhen(io.signal && status === sm.signal_wait_1){
    when(tier_status(0) =/= sm.idole){
      status := sm.wait_tier0_wb
    }.otherwise{
      status := sm.next_tier_is_1
    }
  }.elsewhen(status === sm.wait_tier0_wb && tier_status(0) === sm.idole){
    status := sm.next_tier_is_1
  }.elsewhen(status === sm.wait_tier1_wb && tier_status(1) === sm.idole){
    status := sm.next_tier_is_0
  }.elsewhen(io.end){
    status := sm.idole
  }

  val next_tier_mask = Cat(status === sm.next_tier_is_1 | status === sm.sync_send_0 | status === sm.signal_wait_0 | status === sm.wait_tier1_wb,
      status === sm.next_tier_is_0 | status === sm.sync_send_1 | status === sm.signal_wait_1 | status === sm.wait_tier0_wb)
  val axi = Wire(Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))
  val tier_base_addr = RegInit(VecInit(Seq.fill(2)(0.U(AXI_ADDR_WIDTH.W))))
  val step_fin = io.signal && (status === sm.signal_wait_0 | status === sm.signal_wait_1)
  tier_base_addr.zipWithIndex.map{
    case(a, i) => {
      when((io.start && status === sm.idole) | step_fin){
        a := io.tiers_base_addr(i)
      }.elsewhen(next_tier_mask(i) && axi.aw.ready.asBool() && axi.aw.valid.asBool()){
        a := a + 1024.U
      }.elsewhen(!next_tier_mask(i) && axi.ar.ready.asBool() && axi.ar.valid.asBool()){
        a := a + 1024.U
      }
    }
  }
  tier_counter.zipWithIndex.map{
    case(c, i) => {
      when(next_tier_mask(i) && io.cacheable_in.map{ case i => i.ready && i.valid }.reduce(_|_)){
        c := c + io.cacheable_in.map{ case i => (i.ready && i.valid).asTypeOf(UInt(6.W)) }.reduce(_+_)
      }.elsewhen(!next_tier_mask(i) && io.cacheable_out.ready && io.cacheable_out.valid){
        c := Mux(tier_fifo(i).io.out.data_count > 16.U, c - 16.U, c - tier_fifo(i).io.out.data_count)
      }
    }
  }
  tier_fifo.zipWithIndex.map{
    case(f, i) => {
      f.io.out.rd_en := Mux(next_tier_mask(i), axi.w.ready, io.cacheable_out.ready)
      f.io.out.wr_en := Mux(next_tier_mask(i), 0.U, axi.r.valid)
      f.io.out.din := Mux(next_tier_mask(i), 0.U, axi.r.bits.rdata)
      f.io.flush := io.signal && (status === sm.signal_wait_0 || status === sm.signal_wait_1)
      f.io.in.zipWithIndex.map{
        case(in, x) => {
          in.valid := Mux(next_tier_mask(i), io.cacheable_in(x).valid, false.B)
          in.bits := Mux(next_tier_mask(i), io.cacheable_in(x).bits, 0.U.asTypeOf(in.bits.cloneType))
        }
      }
      f.io.is_current_tier := ~next_tier_mask(i)
    }
  }
  tier_status.zipWithIndex.map{
    case (s, i) => {
      when(next_tier_mask(i) && tier_fifo(i).io.out.full && s === sm.idole){
        s := sm.writeback
      }.elsewhen(!next_tier_mask(i) && tier_fifo(i).is_empty() && tier_counter(i) =/= 0.U && s === sm.idole){
        s := sm.readback
      }.elsewhen(s === sm.writeback && axi.aw.ready){
        s := sm.writebackdata
      }.elsewhen(s === sm.readback && axi.ar.ready){
        s := sm.readbackdata
      }.elsewhen(s === sm.readbackdata && axi.r.bits.rlast.asBool() && axi.r.ready && axi.r.valid){
        s := sm.idole
      }.elsewhen(s === sm.writebackdata && axi.w.bits.wlast.asBool() && axi.w.ready.asBool()){
        s := sm.idole
      }
    }
  }

  io.cacheable_in.zipWithIndex.map{
    case (in, i) => {
      in.ready := Mux(next_tier_mask(0), tier_fifo(0).io.in(i).ready, tier_fifo(1).io.in(i).ready)
    }
  }
  io.unvisited_size := Mux(next_tier_mask(0), tier_counter(0), tier_counter(1))
  io.cacheable_out.valid := MuxCase(false.B,
    Array(next_tier_mask(0) -> (tier_fifo(1).is_valid() | status === sm.sync_send_1),
          next_tier_mask(1) -> (tier_fifo(0).is_valid() | status === sm.sync_send_0)))
  io.cacheable_out.bits.tdata := MuxCase(0.U(512.W),
    Array((status === sm.sync_send_0) -> "x80000000".asUInt(512.W),
          (status === sm.sync_send_1) -> "x80000000".asUInt(512.W),
          next_tier_mask(0) -> tier_fifo(1).io.out.dout,
          next_tier_mask(1) -> tier_fifo(0).io.out.dout))
  io.cacheable_out.bits.tkeep := Mux(status === sm.sync_send_0 | status === sm.sync_send_1, 1.U(16.W),
    VecInit(Seq.tabulate(16)(
      i => Mux(next_tier_mask(0), tier_fifo(1).io.out.data_count > i.U, tier_fifo(0).io.out.data_count > i.U)
    )).asUInt())
  io.cacheable_out.bits.tlast := true.B

  val wcount = RegInit(0.U(8.W))
  when(axi.aw.valid.asBool() && axi.aw.ready.asBool()){
    wcount := 16.U
  }.elsewhen(axi.w.valid.asBool() && axi.w.ready.asBool()){
    wcount := wcount - 1.U
  }
  axi.aw.bits.awaddr := Mux(next_tier_mask(0), tier_base_addr(0), tier_base_addr(1))
  axi.aw.bits.awid := 0.U
  axi.aw.bits.awlen := 15.U
  axi.aw.bits.awlock := 0.U
  axi.aw.bits.awburst := 1.U
  axi.aw.bits.awsize := 6.U
  axi.aw.valid := Mux(next_tier_mask(0), tier_status(0) === sm.writeback, tier_status(1) === sm.writeback)
  axi.ar.bits.araddr := Mux(next_tier_mask(0), tier_base_addr(1), tier_base_addr(0))
  axi.ar.bits.arid := 0.U
  axi.ar.bits.arlen := 15.U
  axi.ar.bits.arburst := 1.U
  axi.ar.bits.arsize := 6.U
  axi.ar.bits.arlock := 0.U
  axi.ar.valid := Mux(next_tier_mask(0), tier_status(1) === sm.readback, tier_status(0) === sm.readback)
  axi.w.bits.wdata := Mux(next_tier_mask(0), tier_fifo(0).io.out.dout, tier_fifo(1).io.out.dout)
  axi.w.valid := Mux(next_tier_mask(0), tier_status(0) === sm.writebackdata, tier_status(1) === sm.writebackdata)
  axi.w.bits.wlast := wcount === 1.U
  axi.w.bits.wstrb := VecInit(Seq.fill(64)(true.B)).asUInt()
  axi.b.ready := true.B
  axi.r.ready := true.B

  io.ddr_out(0) <> axi
  io.ddr_out(1) <> io.non_cacheable_in

  io.signal_ack := status === sm.signal_wait_1 | status === sm.signal_wait_0
}

/*
* traveled_edges: updated every super step
* unvisited_size: unvisited vertex in tiers
* */
class controller (AXI_ADDR_WIDTH : Int = 64, Scatter_num : Int, RegName : Map[String, Int] = Map()) extends Module{
  val io = IO(new Bundle() {
    val data = Output(Vec(32, UInt(32.W)))
    val fin = Input(Vec(Scatter_num, (Bool())))
    val signal = Output(Bool())
    val start = Output(Bool())
    val level = Output(UInt(32.W))
    val unvisited_size = Input(UInt(32.W))
    val traveled_edges = Input(UInt(64.W))
    val config = (new axilitedata(AXI_ADDR_WIDTH))
    val flush_cache = Output(Bool())
    val flush_cache_end = Input(Bool())
    val signal_ack = Input(Bool())
    val performance = Input(Vec(1, Bool()))
  })

  def GetRegByName(name : String): UInt = {
    io.data(RegName(name))
  }
  //0 --- start and end register
  //1 --- embedding_base_addr(31, 0)
  //2 --- embedding_base_addr(63, 32)
  //3 --- edge_base_addr(31, 0)
  //4 --- edge_base_addr(63, 32)
  //5 --- level_base_addr(31, 0)
  //6 --- level_base_addr(63, 32)
  //7 --- root
  //8 --- tier1_base_addr(31, 0)
  //9 --- tier1_base_addr(63, 32)
  //10 --- tier2_base_addr(31, 0)
  //11 --- tier2_base_addr(63, 32)
  //12 --- traveled edges(63, 32)
  //13 --- traveled edges(31, 0)
  //14 --- clock(31, 0)
  //15 --- clock(63, 21)
  val controls = Module(new LookupTable(depth = 32, AXI_ADDR_WIDTH = AXI_ADDR_WIDTH))
  val level = RegInit(0.U(32.W))
  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val exe  = Value(0x1.U) // i "load"  -> 000_0011
    val fin   = Value(0x2.U) // i "imm"   -> 001_0011
    val end = Value(0x3.U)
    val start = Value(0x4.U)
    val flush_cache = Value(0x5.U)
    val write_clock = Value(0x6.U)
  }
  val status = RegInit(sm.idole)
  val start = controls.io.data(0)(0) && !controls.io.data(0)(1)
  val FIN = RegInit(VecInit(Seq.fill(Scatter_num)(false.B)))
  val new_tep = Cat(controls.io.data(12), controls.io.data(13)) + io.traveled_edges
  val counterValue = RegInit(0.U(64.W))

  controls.config <> io.config
  controls.io.writeFlag(0) := status === sm.end | (status === sm.fin && io.signal_ack === true.B) | status === sm.write_clock
  controls.io.wptr(0) := Mux1H(Seq(
    (status === sm.fin) -> 12.U,
    (status === sm.write_clock) -> 14.U,
    (status === sm.end) -> 0.U
  ))
  controls.io.dataIn(0) := Mux1H(Seq(
    (status === sm.fin) -> new_tep(63, 32),
    (status === sm.write_clock) -> counterValue(31, 0),
    (status === sm.end) -> 2.U
  ))
  controls.io.writeFlag(1) := (status === sm.fin && io.signal_ack === true.B) | status === sm.write_clock
  controls.io.wptr(1) := Mux(status === sm.fin, 13.U, 15.U)
  controls.io.dataIn(1) := Mux1H(Seq(
    (status === sm.fin) -> new_tep(31, 0),
    (status === sm.write_clock) -> counterValue(63, 32)
  ))

  when(status === sm.idole && start){
    status := sm.start
  }.elsewhen(status === sm.start){
    status := sm.exe
  }.elsewhen(status === sm.exe && FIN.reduce(_&_)){
    status := sm.fin
  }.elsewhen(status === sm.fin && io.signal_ack === true.B){
    when(io.unvisited_size === 0.U){
      status := sm.write_clock
    }.otherwise{
      status := sm.exe
    }
  }.elsewhen(status === sm.flush_cache && io.flush_cache_end){
    status := sm.end
  }.elsewhen(status === sm.write_clock){
    status := sm.flush_cache
  }/*.elsewhen(status === sm.end){
    status := sm.idole
  }*/

  FIN.zipWithIndex.map{
    case(f, i) => {
      when(io.signal){
        f := false.B
      }.elsewhen(io.fin(i)){
        f := true.B
      }
    }
  }

  when(status === sm.fin && io.signal_ack === true.B){
    level := level + 1.U
  }.elsewhen(status === sm.idole && start){
    level := "xffffffff".asUInt(32.W)
  }

  val global_start = status === sm.fin && io.signal_ack === true.B && level === "xffffffff".asUInt(32.W)
  when(global_start) {
    counterValue := 0.U
  }.otherwise{
    counterValue := counterValue + 1.U
  }

  val performanceValue = RegInit(VecInit(Seq.fill(1)(0.U(64.W))))
  dontTouch(performanceValue)
  io.performance.zipWithIndex.map{
    case (p, i) => {
      when(global_start) {
        performanceValue(i) := 0.U
      }.elsewhen(p){
        performanceValue(i) := performanceValue(i) + 1.U
      }
    }
  }

  io.signal := status === sm.start || (status === sm.fin && io.unvisited_size =/= 0.U)
  io.data := controls.io.data
  io.level := level
  io.start := status === sm.start
  io.flush_cache := status === sm.flush_cache
}