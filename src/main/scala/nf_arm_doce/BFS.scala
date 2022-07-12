package bfs

import chisel3._
import chisel3.experimental.ChiselEnum
import nf_arm_doce._
import chisel3.util._
import utils._

//TODO: rename to cache
class WB_engine(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3,
                FPGA_Num: Int, ID : Int) extends Module{
  val io = IO(new Bundle() {
    val wb_data = Decoupled(new Bundle(){
      val wb_block_index = UInt(12.W)
      val buffer_doutb = UInt(48.W)
    })

    val xbar_in = Flipped(Decoupled(new axisdata(4)))
    val level = Input(UInt(32.W))
    val end = Output(Bool())
    val flush = Input(Bool())
  })
  val buffer = Module(new URAM_cluster(64 * 1024,48))
  val region_counter = Module(new BRAM(1024, 9, "region_counter"))

  def get_level_addr(block_index: UInt, size : UInt) : UInt = {
    Cat(block_index, size.asTypeOf(UInt(6.W)))
  }

  def counter_inc(counter : UInt) : UInt = {
    Mux(counter === 63.U, 0.U, counter + 1.U)
  }

  def vid2addr(vid: UInt) : UInt = {
    (vid(30, log2Ceil(FPGA_Num) + 2) << 2.U).asTypeOf(UInt(AXI_ADDR_WIDTH.W))
  }

  //read region counter
  val vid = io.xbar_in.bits.tdata
  val level = io.level
  val dramaddr = vid2addr(vid)
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
  val wb_block_index = RegInit(0.U(12.W))
  val flush_start = (wb_sm === sm.idole) && io.flush
  val size_b = RegInit(0.U(8.W))
  val wb_start = pipeline_1.io.m_axis.tvalid.asBool() && region_counter_doutb === 63.U  && wb_sm === sm.idole
  pipeline_1.io.m_axis.tready := wb_sm === sm.idole
  region_counter_doutb_forward.io.m_axis.tready := wb_sm === sm.idole

  when(wb_start){
    size_b := 64.U
  }.elsewhen(wb_sm === sm.check_size){
    size_b := region_counter_doutb
  }

  when(wb_start){
    wb_block_index := pipeline_1_out.block_index
  }.elsewhen(flush_start){
    wb_block_index := 0.U
  }.elsewhen(wb_sm === sm.wb_1block && wb_block_index =/= (1024 - 1).U){
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
  }.elsewhen(wb_sm === sm.wb_level && io.wb_data.ready) {
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
    when(wb_block_index =/= (1024 - 1).U){
      wb_sm := sm.check_size
    }.otherwise{
      wb_sm := sm.idole
    }
  }

  when(wb_sm === sm.idole || wb_sm === sm.wb_1block){
    count := 1.U
  }.elsewhen(wb_sm === sm.wb_level && io.wb_data.ready){
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

  io.wb_data.valid := wb_sm === sm.wb_level
  io.wb_data.bits.buffer_doutb := buffer.io.doutb
  io.wb_data.bits.wb_block_index := Cat(wb_block_index(9, 0), ID.U(2.W))
  io.end := wb_sm === sm.wb_1block && wb_block_index === (1024 - 1).U
}

class Apply(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3,
            FPGA_Num: Int) extends Module{
  val io = IO(new Bundle(){
    val ddr_aw = Decoupled(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Decoupled(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(Decoupled(new axib(AXI_ID_WIDTH, 1)))
    val gather_in = Flipped(Decoupled(new axisdata(AXI_DATA_WIDTH)))

    //control path
    val level = Input(UInt(32.W))
    val level_base_addr = Input(UInt(64.W))
    val end = Output(Bool())
    val flush = Input(Bool())
  })

  io.ddr_aw.bits.tie_off(0.U)
  io.ddr_w.bits.tie_off()
  io.ddr_w.valid := false.B
  io.ddr_aw.valid := false.B
  io.ddr_b.ready := true.B
  io.gather_in.ready := true.B

  io.end := io.flush
}

class Gather(AXI_DATA_WIDTH: Int = 64, Broadcast_Num: Int) extends Module{
  val io = IO(new Bundle() {
    val ddr_in = Flipped(Decoupled(new axisdata(AXI_DATA_WIDTH, 4)))
    val gather_out = Vec(Broadcast_Num, Decoupled(new axisdata(4, 4)))
    val level_cache_out = Decoupled(new axisdata(AXI_DATA_WIDTH, 4))
  })
  val broadcaster = Module(new axis_broadcaster(AXI_DATA_WIDTH, (1 + Broadcast_Num), "gather_broadcaster"))
  broadcaster.io.s_axis.connectfrom(io.ddr_in.bits)
  broadcaster.io.s_axis.tvalid := io.ddr_in.valid
  io.ddr_in.ready := broadcaster.io.s_axis.tready
  broadcaster.io.aresetn := ~reset.asBool()
  broadcaster.io.aclk := clock.asBool()

  val v2Apply_fifo = Module(new axis_reg_slice(AXI_DATA_WIDTH, "v2A_reg_slice"))
  v2Apply_fifo.io.aclk := clock.asBool()
  v2Apply_fifo.io.aresetn := ~reset.asBool()
  v2Apply_fifo.io.s_axis.tdata := broadcaster.io.m_axis.tdata(AXI_DATA_WIDTH * 8 * (Broadcast_Num + 1) - 1, AXI_DATA_WIDTH * 8 * Broadcast_Num)
  v2Apply_fifo.io.s_axis.tvalid := broadcaster.io.m_axis.tvalid(Broadcast_Num)
  v2Apply_fifo.io.s_axis.tkeep := broadcaster.io.m_axis.tkeep(AXI_DATA_WIDTH * (Broadcast_Num + 1) - 1, AXI_DATA_WIDTH * Broadcast_Num)
  v2Apply_fifo.io.s_axis.tlast := broadcaster.io.m_axis.tlast(Broadcast_Num)

  val v2Broadcast_fifo = Seq.tabulate(Broadcast_Num)(
    i => Module(new axis_reg_slice(AXI_DATA_WIDTH / Broadcast_Num, "v2B_reg_slice"))
  )
  v2Broadcast_fifo.zipWithIndex.map{
    case (b, i) => {
      b.io.aresetn := ~reset.asBool()
      b.io.aclk := clock.asBool()
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

  io.level_cache_out.valid := v2Apply_fifo.io.m_axis.tvalid
  v2Apply_fifo.io.m_axis.tready := io.level_cache_out.ready
  v2Apply_fifo.io.m_axis.connectto(io.level_cache_out.bits, 0)

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
    val xbar_out = Decoupled(new axisdata(AXI_DATA_WIDTH, 4))
    //control signal
    val edge_base_addr = Input(UInt(64.W))
    //val free_ptr = Input(UInt((AXI_ID_WIDTH-1).W))
    //val read_edge_num = Output(UInt(32.W))
    val read_edge_fifo_empty = Output(Bool())
    val credit = Output(UInt(8.W))
    val credit_dec = Input(Bool())
    val traveled_edges = Output(UInt(64.W))
    val signal = Input(Bool())
    //val inflight_vtxs = Input(UInt(64.W))
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

  class request extends Bundle() {
    val count = UInt(32.W)
    val is_new = Bool()
    val index = UInt(32.W)
    val reg_ptr = UInt(6.W)
  }

  //tag return data
  val KEEP_WIDTH = AXI_DATA_WIDTH / 4
  object sm extends ChiselEnum {
    //val idole = Value(0x0.U)
    val firstBurst  = Value(0x0.U) // i "load"  -> 000_0011
    val remainingBurst_embedding   = Value(0x1.U) // i "imm"   -> 001_0011
    val remainingBurst_edge   = Value(0x2.U)
  }
  val status = RegInit(sm.firstBurst)
  val num = RegInit(0.U(32.W))
  when(status === sm.firstBurst && io.in.valid && io.in.ready){
    when(io.in.bits.rlast.asBool()){
      status := sm.firstBurst
    }.elsewhen(get_flag(io.in.bits.rid)){
      status := sm.remainingBurst_edge
    }.otherwise{
      status := sm.remainingBurst_embedding
    }
  }.elsewhen((status === sm.remainingBurst_edge  || status === sm.remainingBurst_embedding)
    && io.in.valid && io.in.ready && io.in.bits.rlast.asBool()){
    status := sm.firstBurst
  }
  val traveled_edges_reg = RegInit(0.U(64.W))
  when(io.signal){
    traveled_edges_reg := 0.U
  }.elsewhen(io.in.valid && io.in.ready && !get_flag(io.in.bits.rid)
    && status === sm.firstBurst){
    traveled_edges_reg := traveled_edges_reg + get_edge_count(io.in.bits.rdata)
  }
  io.traveled_edges := traveled_edges_reg

  //convert axi read channel to axis
  val num_regfile = Module(new regFile(32, 64))
  when(status === sm.firstBurst
    && io.in.valid && io.in.ready && !io.in.bits.rlast.asBool()){
    when(get_flag(io.in.bits.rid)){
      num := Mux(get_edge_count(num_regfile.io.dataOut) > 4.U, get_edge_count(num_regfile.io.dataOut) - 4.U, 0.U)
    }.otherwise{
      num := Mux(get_edge_count(io.in.bits.rdata) > 2.U,
        get_edge_count(io.in.bits.rdata) - 2.U, 0.U)
    }
  }.elsewhen(status === sm.remainingBurst_edge && io.in.valid && io.in.ready){
    when(io.in.bits.rlast.asBool()){
      num := 0.U
    }.otherwise{
      num := num - KEEP_WIDTH.U
    }
  }.elsewhen(status === sm.remainingBurst_embedding && io.in.valid && io.in.ready){
    when(io.in.bits.rlast.asBool()){
      num := 0.U
    }.otherwise{
      num := Mux(num > KEEP_WIDTH.U, num - KEEP_WIDTH.U, 0.U)
    }
  }

  val keep = Wire(Vec(KEEP_WIDTH, Bool()))
  keep.zipWithIndex.map {
    case(k, i) => {
      k := false.B
      when(status === sm.firstBurst && get_flag(io.in.bits.rid)){
        k := get_edge_count(num_regfile.io.dataOut) > i.U
      }.elsewhen(status === sm.firstBurst && !get_flag(io.in.bits.rid)){
        if(i <= 1){
          k := false.B
        }else{
          k := get_edge_count(io.in.bits.rdata) > (i - 2).U
        }
      }.elsewhen(status === sm.remainingBurst_edge || status === sm.remainingBurst_embedding){
        k := num > i.U
      }
    }
  }
  io.xbar_out.valid := io.in.valid && io.in.ready
  io.xbar_out.bits.tdata := io.in.bits.rdata
  io.xbar_out.bits.tlast := io.in.bits.rlast
  io.xbar_out.bits.tkeep := keep.asUInt()

  //buffer read edge array requests
  val edge_read_buffer = Module(new BRAM_fifo(32, 72, "meta_fifo"))
  val max_vertex_read_per_transaction = 64.U
  val edge_read_buffer_din = Wire(new request())
  edge_read_buffer.io.clk := clock.asBool()
  edge_read_buffer.io.srst := reset.asBool()
  edge_read_buffer_din.count := get_edge_count(io.in.bits.rdata(63, 0)) - EMBEDDING.asUInt()
  edge_read_buffer_din.index := get_edge_array_index(io.in.bits.rdata(63, 0))
  edge_read_buffer_din.is_new := true.B
  edge_read_buffer_din.reg_ptr := 0.U
  edge_read_buffer.io.wr_en := false.B
  edge_read_buffer.io.rd_en := false.B
  edge_read_buffer.io.din := edge_read_buffer_din.asUInt()
  when(io.in.valid && io.in.ready & (status === sm.firstBurst)){
    edge_read_buffer.io.wr_en := true.B
    when(get_flag(io.in.bits.rid)){
      edge_read_buffer_din.is_new := false.B
      edge_read_buffer_din.reg_ptr := get_ptr(io.in.bits.rid)
      when(get_edge_count(num_regfile.io.dataOut) > max_vertex_read_per_transaction){
        edge_read_buffer_din.count :=
          get_edge_count(num_regfile.io.dataOut) - max_vertex_read_per_transaction
        edge_read_buffer_din.index :=
          get_edge_array_index(num_regfile.io.dataOut) + max_vertex_read_per_transaction
      }.otherwise{
        edge_read_buffer_din.count := 0.U
      }
    }.otherwise{
      when(get_edge_count(io.in.bits.rdata) < EMBEDDING.asUInt()){
        edge_read_buffer_din.count := 0.U
      }
    }
  }
  io.in.ready := (edge_read_buffer.io.full === false.B | status =/= sm.firstBurst) && io.xbar_out.ready

  //handle requests
  val edge_read_buffer_dout =  edge_read_buffer.io.dout.asTypeOf(new request())
  val remainning_edges = edge_read_buffer_dout.count
  val min_credit_for_expand = 4.U
  object cache_sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val expand_old_request  = Value(0x1.U) // i "load"  -> 000_0011
    val new_request = Value(0x2.U)
    val update_request = Value(0x3.U)
    val rm_request = Value(0x4.U)
    val expand_new_request = Value(0x5.U)
    val fill_free_queue = Value(0x6.U)
    val release_credit = Value(0x7.U)
    val expand_fin = Value(0x8.U)
    val reset = Value(0x9.U)
  }
  val cache_status = RegInit(cache_sm.reset)
  val free_queue = Module(new BRAM_fifo(32, 6, "free_queue"))
  val init_seq = RegInit(0.U(6.W))
  when(cache_status === cache_sm.fill_free_queue){
    init_seq := init_seq + 1.U
  }
  val expand_index = RegInit(0.U(32.W))
  val expand_count = RegInit(0.U(32.W))
  val credit = RegInit(32.U(32.W))
  when(cache_status === cache_sm.reset && init_seq === 0.U){
    cache_status := cache_sm.fill_free_queue
  }.elsewhen(cache_status === cache_sm.fill_free_queue && init_seq === 31.U){
    cache_status := cache_sm.idole
  }.elsewhen(cache_status === cache_sm.idole && edge_read_buffer.is_valid()){
    when(edge_read_buffer_dout.is_new === true.B){
      when(remainning_edges === 0.U){
        cache_status := cache_sm.release_credit
      }.otherwise{
        when(remainning_edges > max_vertex_read_per_transaction && credit > min_credit_for_expand){
          cache_status := cache_sm.expand_new_request
          expand_index := edge_read_buffer_dout.index
          expand_count := edge_read_buffer_dout.count
        }.otherwise{
          cache_status := cache_sm.new_request
        }
      }
    }.otherwise{
      when(remainning_edges === 0.U){
        cache_status := cache_sm.rm_request
      }.otherwise{
        when(remainning_edges > max_vertex_read_per_transaction && credit > min_credit_for_expand){
          cache_status := cache_sm.expand_old_request
          expand_index := edge_read_buffer_dout.index
          expand_count := edge_read_buffer_dout.count
        }.otherwise{
          cache_status := cache_sm.update_request
        }
      }
    }
  }.elsewhen(io.out.ready && io.out.valid && cache_status === cache_sm.new_request){
    cache_status := cache_sm.idole
    edge_read_buffer.io.rd_en := true.B
  }.elsewhen(io.out.ready && io.out.valid && cache_status === cache_sm.update_request){
    cache_status := cache_sm.idole
    edge_read_buffer.io.rd_en := true.B
  }.elsewhen(cache_status === cache_sm.release_credit){
    cache_status := cache_sm.idole
    edge_read_buffer.io.rd_en := true.B
  }.elsewhen(cache_status === cache_sm.rm_request){
    cache_status := cache_sm.idole
    edge_read_buffer.io.rd_en := true.B
  }.elsewhen(io.out.ready && io.out.valid && cache_status === cache_sm.expand_new_request){
    expand_index := expand_index + max_vertex_read_per_transaction
    expand_count := expand_count - max_vertex_read_per_transaction
    when(credit <= min_credit_for_expand || expand_count <= (max_vertex_read_per_transaction << 1.U).asUInt()){
      cache_status := cache_sm.expand_fin
    }
  }.elsewhen(io.out.ready && io.out.valid && cache_status === cache_sm.expand_old_request){
    expand_index := expand_index + max_vertex_read_per_transaction
    expand_count := expand_count - max_vertex_read_per_transaction
    when(credit <= min_credit_for_expand || expand_count <= (max_vertex_read_per_transaction << 1.U).asUInt()){
      cache_status := cache_sm.expand_fin
    }.otherwise{
      cache_status := cache_sm.expand_new_request
    }
  }.elsewhen(io.out.ready && io.out.valid && cache_status === cache_sm.expand_fin){
    cache_status := cache_sm.idole
    edge_read_buffer.io.rd_en := true.B
  }

  //send axi read requests
  val num_vertex = MuxCase(max_vertex_read_per_transaction, Array(
    (cache_status === cache_sm.expand_fin && expand_count <= max_vertex_read_per_transaction) ->
      expand_count,
    (remainning_edges <= max_vertex_read_per_transaction) -> (remainning_edges)
  ))
  var arlen = Mux(((num_vertex >> log2Ceil(AXI_DATA_WIDTH/4)) << log2Ceil(AXI_DATA_WIDTH/4)).asUInt() < num_vertex,
    (num_vertex >> log2Ceil(AXI_DATA_WIDTH/4)),
    (num_vertex >> log2Ceil(AXI_DATA_WIDTH/4)).asUInt() - 1.U)
  io.out.bits.araddr :=
    io.edge_base_addr +
      (Mux(cache_status === cache_sm.expand_fin | cache_status === cache_sm.expand_new_request |
        cache_status === cache_sm.expand_old_request,
        expand_index, edge_read_buffer_dout.index) << 2.U).asUInt()
  io.out.valid := cache_status === cache_sm.new_request | cache_status === cache_sm.update_request |
    cache_status === cache_sm.expand_fin | cache_status === cache_sm.expand_new_request |
    cache_status === cache_sm.expand_old_request
  io.out.bits.arlen := arlen.asUInt()
  io.out.bits.arburst := 1.U(2.W)
  io.out.bits.arlock := 0.U
  io.out.bits.arsize := MuxCase(log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W), Seq.tabulate(log2Ceil(AXI_DATA_WIDTH/4))(
    x => ((num_vertex <= Seq.tabulate(log2Ceil(AXI_DATA_WIDTH/4))(i => 1 << i)(x).asUInt()) -> (x + 2).asUInt())
  ))
  io.out.bits.arid := Cat(1.U(1.W),
    Mux(cache_status === cache_sm.new_request | cache_status === cache_sm.expand_new_request |
      cache_status === cache_sm.expand_fin, free_queue.io.dout.asTypeOf(UInt((AXI_ID_WIDTH- 1).W)),
      edge_read_buffer_dout.reg_ptr.asTypeOf(UInt((AXI_ID_WIDTH- 1).W))))

  //buffer read request info in num_regfile
  num_regfile.io.writeFlag := false.B
  num_regfile.io.wptr := 0.U
  num_regfile.io.dataIn := 0.U
  num_regfile.io.rptr := get_ptr(io.in.bits.rid)
  free_queue.io.clk := clock.asBool()
  free_queue.io.srst := reset.asBool()
  free_queue.io.rd_en := false.B
  free_queue.io.din := 0.U
  free_queue.io.wr_en := false.B
  when(cache_status === cache_sm.new_request){
    num_regfile.io.wptr := free_queue.io.dout
    free_queue.io.rd_en := io.out.ready && io.out.valid
    num_regfile.io.writeFlag := free_queue.is_valid() && io.out.ready && io.out.valid
    num_regfile.io.dataIn := Cat(edge_read_buffer_dout.count,
      edge_read_buffer_dout.index)
  }.elsewhen(cache_status === cache_sm.update_request){
    num_regfile.io.wptr := edge_read_buffer_dout.reg_ptr
    num_regfile.io.writeFlag := io.out.ready && io.out.valid
    num_regfile.io.dataIn := Cat(edge_read_buffer_dout.count,
      edge_read_buffer_dout.index)
  }.elsewhen(cache_status === cache_sm.rm_request){
    free_queue.io.din := edge_read_buffer_dout.reg_ptr
    free_queue.io.wr_en := true.B
  }.elsewhen(cache_status === cache_sm.fill_free_queue){
    free_queue.io.din :=  init_seq
    free_queue.io.wr_en := true.B
  }.elsewhen(cache_status === cache_sm.expand_new_request){
    num_regfile.io.wptr := free_queue.io.dout
    free_queue.io.rd_en := io.out.ready && io.out.valid
    num_regfile.io.writeFlag := free_queue.is_valid() && io.out.ready && io.out.valid
    num_regfile.io.dataIn := Cat(max_vertex_read_per_transaction,
      expand_index)
  }.elsewhen(cache_status === cache_sm.expand_old_request){
    num_regfile.io.wptr := edge_read_buffer_dout.reg_ptr
    num_regfile.io.writeFlag := io.out.ready && io.out.valid
    num_regfile.io.dataIn := Cat(max_vertex_read_per_transaction,
      expand_index)
  }.elsewhen(cache_status === cache_sm.expand_fin){
    num_regfile.io.wptr := free_queue.io.dout
    free_queue.io.rd_en := io.out.ready && io.out.valid
    num_regfile.io.writeFlag := free_queue.is_valid() && io.out.ready && io.out.valid
    num_regfile.io.dataIn := Cat(expand_count,
      expand_index)
  }

  io.read_edge_fifo_empty := edge_read_buffer.is_empty() && cache_status === cache_sm.idole

  val credit_dec = io.out.ready && io.out.valid && cache_status === cache_sm.expand_new_request |
    io.out.ready && io.out.valid && cache_status === cache_sm.expand_old_request |
    io.credit_dec
  when(credit_dec && cache_status =/= cache_sm.release_credit &&
    cache_status =/= cache_sm.rm_request){
    credit := credit - 1.U
  }.elsewhen(!credit_dec && (cache_status === cache_sm.release_credit ||
    cache_status === cache_sm.rm_request)){
    credit := credit + 1.U
  }
  io.credit := credit
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
    io.embedding_base_addr + (vid(30, log2Ceil(FPGA_Num)) << log2Ceil(4 * EMBEDDING + 8)).asUInt()
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

  //read offset and edge embedding array
  val vertex_read_buffer = Module(new BRAM_fifo(32, 32, "vid_fifo"))
  vertex_read_buffer.io.clk := clock.asBool()
  vertex_read_buffer.io.srst := reset.asBool()
  vertex_read_buffer.io.wr_en := io.gather_in.valid
  vertex_read_buffer.io.din := io.gather_in.bits.tdata
  io.gather_in.ready := vertex_read_buffer.io.full === false.B

  val edge_cache = Module(new readEdge_engine(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, EMBEDDING,
    Broadcast_Num, Broadcast_index))
  edge_cache.io.edge_base_addr := io.edge_base_addr
  edge_cache.io.in <> io.ddr_r
  io.traveled_edges := edge_cache.io.traveled_edges
  edge_cache.io.signal := io.signal

  val arbi = Module(new AMBA_Arbiter(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1), 2))
  val vertex_read_id = RegInit(0.U((AXI_ID_WIDTH- 1).W))
  when(arbi.io.in(1).valid && arbi.io.in(1).ready){
    vertex_read_id := vertex_read_id + 1.U
  }
  io.ddr_ar <> arbi.io.out
  arbi.io.in(1).bits.araddr :=  vid2addr(vertex_read_buffer.io.dout)
  arbi.io.in(1).valid :=  vertex_read_buffer.is_valid() && vertex_read_buffer.io.dout(31) === 0.U &&
    edge_cache.io.credit =/= 0.U
  arbi.io.in(1).bits.arlen := ((4 * EMBEDDING + 8) / AXI_DATA_WIDTH - 1).asUInt()
  arbi.io.in(1).bits.arburst := 1.U(2.W)
  arbi.io.in(1).bits.arlock := 0.U
  arbi.io.in(1).bits.arsize :=  log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W)
  arbi.io.in(1).bits.arid :=  Cat(0.U(1.W), vertex_read_id)
  vertex_read_buffer.io.rd_en := (arbi.io.in(1).ready && edge_cache.io.credit =/= 0.U) | upward_status === upward_sm.output_fin
  arbi.io.in(0) <> edge_cache.io.out
  edge_cache.io.credit_dec := arbi.io.in(1).valid && arbi.io.in(1).ready

  val (ar_ready_counter, ar_b_1) = Counter(io.ddr_r.valid === false.B,
    0xffffffff)
  dontTouch(ar_ready_counter)

  //back end of down ward
  val KEEP_WIDTH = AXI_DATA_WIDTH / 4
  val vertex_out_fifo = Module(new axis_data_count_fifo(AXI_DATA_WIDTH, "edge_fifo"))
  vertex_out_fifo.io.s_axis_aresetn := !reset.asBool()
  vertex_out_fifo.io.s_axis_aclk := clock.asBool()
  vertex_out_fifo.io.m_axis.connectto(io.xbar_out.bits, 0)
  io.xbar_out.valid := vertex_out_fifo.io.m_axis.tvalid
  vertex_out_fifo.io.m_axis.tready := io.xbar_out.ready
  vertex_out_fifo.io.s_axis.tvalid := edge_cache.io.xbar_out.valid | upward_status === upward_sm.output_fin | io.start
  vertex_out_fifo.io.s_axis.tdata := MuxCase(edge_cache.io.xbar_out.bits.tdata, Array(
    io.start -> io.root,
    (upward_status === upward_sm.output_fin) -> "x80000000".asUInt((AXI_DATA_WIDTH * 8).W)
  ))
  vertex_out_fifo.io.s_axis.tkeep := MuxCase(edge_cache.io.xbar_out.bits.tkeep, Array(
    io.start -> 0x1.U(KEEP_WIDTH.W),
    (upward_status === upward_sm.output_fin) -> 0x1.U(KEEP_WIDTH.W)
  ))
  vertex_out_fifo.io.s_axis.tlast := true.B
  edge_cache.io.xbar_out.ready := vertex_out_fifo.io.s_axis.tready

  val (edge_fifo_ready_counter, b_1) = Counter(vertex_out_fifo.io.s_axis.tready === 0.U, 0x10000000)
  dontTouch(edge_fifo_ready_counter)

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

  val (ready_counter, b) = Counter(upward_status === upward_sm.sync, 0x10000000)
  dontTouch(ready_counter)

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
    val local_fpga_id = Input(UInt((log2Ceil(FPGA_Num) + 1).W))
    val graph_scale = Input(UInt(32.W))
  })
  val vertex_num = AXI_DATA_WIDTH / 4
  val bitmap = Module(new BRAM(116 * 1024 * 16 / Apply_num, 9, "bitmap_0"))
  val arbi = Module(new axis_arbitrator(4, vertex_num, 4))
  val local_fpga_id = RegInit(0.U((log2Ceil(FPGA_Num)+1).W))
  local_fpga_id := io.local_fpga_id

  def vid_to_sid(vid: UInt, sid: UInt) : Bool = {
    if(FPGA_Num == 1) {
      vid(log2Ceil(Apply_num) - 1, 0) === sid
    }else{
      vid(log2Ceil(Apply_num) + log2Ceil(FPGA_Num) - 1, log2Ceil(FPGA_Num)) === sid &&
        vid(log2Ceil(FPGA_Num) - 1, 0) === local_fpga_id
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

  val scatter_in = Module(new axis_reg_slice(AXI_DATA_WIDTH, "Scatter_in_reg_slice"))
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

  val vertex_in_fifo = Module(new axis_data_fifo(AXI_DATA_WIDTH, "vid_32_fifo"))
  vertex_in_fifo.io.s_axis_aclk := clock.asBool()
  vertex_in_fifo.io.s_axis_aresetn := !reset.asBool()
  vertex_in_fifo.io.s_axis.tdata := scatter_in.io.m_axis.tdata
  vertex_in_fifo.io.s_axis.tlast := scatter_in.io.m_axis.tlast
  vertex_in_fifo.io.s_axis.tvalid := scatter_in.io.m_axis.tvalid & filtered_keep.reduce(_|_)
  vertex_in_fifo.io.s_axis.tkeep := filtered_keep.asUInt()
  scatter_in.io.m_axis.tready := vertex_in_fifo.io.s_axis.tready

  vertex_in_fifo.io.m_axis.connectto(arbi.io.xbar_in.bits, 0)
  arbi.io.xbar_in.valid := vertex_in_fifo.io.m_axis.tvalid
  vertex_in_fifo.io.m_axis.tready := arbi.io.xbar_in.ready

  val vertex_out_fifo = Module(new axis_reg_slice(4, "scatter_out_reg_slice"))
  vertex_out_fifo.io.aclk := clock.asBool()
  vertex_out_fifo.io.aresetn := ~reset.asBool()

  //read or write bitmap ehnr this is not FIN
  val bitmap_arvalid = arbi.io.ddr_out.valid
  val halt = vertex_out_fifo.io.s_axis.tready === false.B
  val bitmap_wait = RegInit(0.U.asTypeOf(ValidIO(UInt(32.W))))
  when(!halt){
    bitmap_wait.valid := bitmap_arvalid
    bitmap_wait.bits := arbi.io.ddr_out.bits.tdata
  }
  val bitmap_doutb = Wire(UInt(9.W))
  val bitmap_write_addr = RegInit(0.U.asTypeOf(ValidIO(UInt(32.W))))
  when(!halt) {
    bitmap_write_addr.valid := bitmap_wait.valid &&
      (bitmap_doutb(vid2bitmap_offset(bitmap_wait.bits)) =/= 1.U(1.W) | bitmap_wait.bits(31) === 1.U(1.W))
    bitmap_write_addr.bits := bitmap_wait.bits
  }
  val bitmap_write_data = RegInit(0.U.asTypeOf(ValidIO(UInt(9.W))))
  when(!halt){
    bitmap_write_data.valid := bitmap_wait.valid &&
      (bitmap_doutb(vid2bitmap_offset(bitmap_wait.bits)) =/= 1.U(1.W) | bitmap_wait.bits(31) === 1.U(1.W))
    bitmap_write_data.bits := bitmap_doutb | (1.U(9.W) << vid2bitmap_offset(bitmap_wait.bits)).asUInt()
  }
  val bitmap_write_data_forward = RegInit(0.U.asTypeOf(ValidIO(UInt(9.W))))
  bitmap_write_data_forward.valid := bitmap_write_addr.valid && bitmap_arvalid &&
    (vid2bitmap_addr(arbi.io.ddr_out.bits.tdata) === vid2bitmap_addr(bitmap_write_addr.bits))
  bitmap_write_data_forward.bits := bitmap_write_data.bits
  bitmap_doutb := MuxCase(bitmap.io.doutb, Array(
    (bitmap_write_addr.valid && bitmap_wait.valid &&
      vid2bitmap_addr(bitmap_wait.bits) === vid2bitmap_addr(bitmap_write_addr.bits)) -> bitmap_write_data.bits,
    bitmap_write_data_forward.valid -> bitmap_write_data_forward.bits
  ))

  arbi.io.ddr_out.ready := !halt
  bitmap.io.enb :=  true.B//bitmap_arvalid
  bitmap.io.addrb := Mux(halt, vid2bitmap_addr(bitmap_wait.bits) ,vid2bitmap_addr(arbi.io.ddr_out.bits.tdata))
  bitmap.io.clkb := clock.asBool()
  vertex_out_fifo.io.s_axis.tvalid := bitmap_write_addr.valid
  vertex_out_fifo.io.s_axis.tdata := bitmap_write_addr.bits
  bitmap.io.ena := true.B
  bitmap.io.wea := bitmap_write_addr.valid & bitmap_write_addr.bits(31) === 0.U(1.W)
  bitmap.io.clka := clock.asBool()
  bitmap.io.dina := bitmap_write_data.bits
  bitmap.io.addra := vid2bitmap_addr(bitmap_write_addr.bits)
  vertex_out_fifo.io.m_axis.tready := true.B
  io.ddr_out.valid := false.B
  io.ddr_out.bits.tkeep := true.B
  io.ddr_out.bits.tlast := true.B
  io.ddr_out.bits.tdata := vertex_out_fifo.io.m_axis.tdata

  //control path
  val wb_bram = RegInit(0.U(32.W))

  val aligned_graph_scale = (io.graph_scale >> 19.U).asUInt()

  val num_small_sp = RegInit(0.U(32.W))
  when(vertex_out_fifo.io.m_axis.tvalid.asBool() & vertex_out_fifo.io.m_axis.tdata(31) === 1.U){
    num_small_sp := aligned_graph_scale +
      (io.graph_scale > (aligned_graph_scale << 19.U).asUInt()).asUInt()
  }.elsewhen(wb_bram > 0.U && wb_bram <= 64.U && num_small_sp =/= 0.U){
    num_small_sp := num_small_sp - 1.U
  }
  val vertices_per_sp = Mux(num_small_sp === 1.U, io.graph_scale(19,0), (1<<19).U(32.W))
  when(num_small_sp =/= 0.U && wb_bram === 0.U) {
    when(num_small_sp === 1.U){
      wb_bram := (vertices_per_sp << 2.U(4.W)).asUInt() + 0x4000.U(32.W) //float operations latency
    }.elsewhen(num_small_sp === 2.U){
      wb_bram := ((vertices_per_sp + io.graph_scale(19,0)) << 2.U(4.W)).asUInt() + 0x4000.U(32.W)
    }.otherwise{
      //load next small sp block.
      wb_bram := (vertices_per_sp << 3.U(4.W)).asUInt() + 0x4000.U(32.W) //float operations latency
    }
  }.elsewhen(wb_bram > 0.U){
    wb_bram := Mux(wb_bram > 64.U, wb_bram - 64.U, 0.U)
  }
  io.end := wb_bram > 0.U && wb_bram <= 64.U && num_small_sp === 1.U

  val ready_counter = RegInit(0.U(32.W))
  dontTouch(ready_counter)
  when(io.xbar_in.ready === false.B && io.xbar_in.valid === true.B){
    ready_counter := ready_counter + 1.U
  }
}

//TODO: Try to use fifoIO
class multi_channel_fifo(AXI_DATA_WIDTH: Int = 64, size : Int, val Scatter_num : Int) extends Module{
  val io = IO(new Bundle() {
    val in = Flipped(Decoupled(Vec(Scatter_num, new axisdata(4, 4))))
    val out = new Bundle() {
      val almost_full = Output(Bool())
      val din = Input(UInt((8 * AXI_DATA_WIDTH).W))
      val wr_en = Input(Bool())
      val dout = Output(UInt((8 * AXI_DATA_WIDTH).W))
      val rd_en = Input(Bool())
      val data_count = Output(UInt((log2Ceil(size * AXI_DATA_WIDTH / 4) + 1).W))
      val valid = Output(Bool())
    }
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
    i => Module(new BRAM_fifo(size, 32, ("collector_fifo_0")))
  )
  val collector_data = Wire(Vec(16, UInt(32.W)))
  io.in.ready := Seq.tabulate(16)(i => collector_fifos(i).io.full === false.B).reduce(_&_)
  collector_fifos.zipWithIndex.map{
    case (f, i) => {
      f.io.din := Mux(io.is_current_tier, io.out.din(32 * (i + 1) - 1, 32 * i), io.in.bits(i).tdata)
      f.io.wr_en := Mux(io.is_current_tier, io.out.wr_en ,io.in.valid & io.in.bits(i).tkeep)
      collector_data(i) := f.io.dout
      f.io.clk := clock.asBool()
      f.io.srst := reset.asBool()
      f.io.rd_en := io.out.rd_en
    }
  }

  io.out.valid := Seq.tabulate(16)(i => collector_fifos(i).is_valid()).reduce(_|_)
  io.out.dout := collector_data.asUInt()
  io.out.data_count := collector_fifos.map{
    i => i.io.data_count.asTypeOf(UInt((log2Ceil(size * AXI_DATA_WIDTH / 4) + 1).W))
  }.reduce(_+_)
  io.out.almost_full := collector_fifos.map{
    i => (i.io.data_count > (size / 2).U)
  }.reduce(_&_)
}

/*
* mc send FIN when no more data in current tier FIFO
* FIN: vid[31] = 1
* */
class multi_port_mc(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int, AXI_SIZE_WIDTH: Int = 3,
                    Scatter_num : Int, FPGA_Num: Int) extends Module {
  val io = IO(new Bundle() {
    val cacheable_out = Decoupled(new axisdata(AXI_DATA_WIDTH, 4))
    val cacheable_in = Vec(Scatter_num, Flipped(Decoupled(new axisdata(4, 4))))
    val non_cacheable_in = new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)
    val ddr_out = Vec(2, Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))

    //control path
    val graph_scale = Input(UInt(32.W))
    val unvisited_size = Output(UInt(32.W))
    val start = Input(Bool())
    val signal = Input(Bool())
    val end = Input(Bool())
    val signal_ack = Output(Bool())
  })

  val tier_counter = RegInit(0.U(32.W))
  val tier_data = RegInit(VecInit.tabulate(16)(
    x => (x << log2Ceil(FPGA_Num)).U(32.W)
  ))
  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val next_tier_is_0  = Value(0x1.U)
    val sync_send_0 = Value(0x5.U)
    val signal_wait_0 = Value(0x7.U)
  }
  val status = RegInit(sm.idole)

  tier_data.zipWithIndex.map{
    case(d, i) => {
      when(status === sm.next_tier_is_0 && io.cacheable_out.valid && io.cacheable_out.ready){
        d := d + (16 << log2Ceil(FPGA_Num)).U
      }.elsewhen(io.signal && status === sm.signal_wait_0){
        d := (i << log2Ceil(FPGA_Num)).U(32.W)
      }
    }
  }

  when(io.start && status === sm.idole){
    tier_counter := io.graph_scale
  }.elsewhen(status === sm.next_tier_is_0 && io.cacheable_out.valid && io.cacheable_out.ready){
    tier_counter := Mux(tier_counter > 16.U, tier_counter - 16.U, 0.U)
  }.elsewhen(io.signal && status === sm.signal_wait_0){
    tier_counter := io.graph_scale
  }
  val keep = VecInit(Seq.tabulate(16)(x => Mux(x.U < tier_counter, true.B, false.B)))

  when(io.start && status === sm.idole){
    status := sm.next_tier_is_0
  }.elsewhen(status === sm.next_tier_is_0 && tier_counter <= 16.U && tier_counter > 0.U &&
    io.cacheable_out.valid && io.cacheable_out.ready){
    status := sm.sync_send_0
  }.elsewhen(status === sm.sync_send_0 && io.cacheable_out.valid && io.cacheable_out.ready){
    status := sm.signal_wait_0
  }.elsewhen(io.signal && status === sm.signal_wait_0){
      status := sm.next_tier_is_0
  }.elsewhen(io.end){
    status := sm.idole
  }

  val axi = Wire(Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))

  io.unvisited_size := 0.U
  io.cacheable_out.valid := status === sm.sync_send_0 || status === sm.next_tier_is_0
  io.cacheable_out.bits.tdata := Mux(status === sm.sync_send_0, "x80000000".asUInt(512.W),
    tier_data.asUInt())
  io.cacheable_out.bits.tkeep := Mux(status === sm.sync_send_0, 1.U(16.W), keep.asUInt())
  io.cacheable_out.bits.tlast := true.B

  axi.aw.bits.tie_off(0.U)
  axi.aw.valid := false.B
  axi.ar.bits.tie_off(0.U)
  axi.ar.valid := 0.U
  axi.w.valid := false.B
  axi.w.bits.tie_off()
  axi.b.ready := true.B
  axi.r.ready := true.B

  io.ddr_out(0) <> axi
  io.ddr_out(1) <> io.non_cacheable_in
  io.cacheable_in.map{
    case x => x.ready := true.B
  }
  io.signal_ack := status === sm.signal_wait_0
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
    when(GetRegByName("LoopNum") === level){
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

  io.signal := status === sm.start || (status === sm.fin && GetRegByName("LoopNum") =/= level)
  io.data := controls.io.data
  io.level := level
  io.start := status === sm.start
  io.flush_cache := status === sm.flush_cache
}