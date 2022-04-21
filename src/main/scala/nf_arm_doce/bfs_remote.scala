package remote

import chisel3._
import chisel3.experimental.ChiselEnum
import chisel3.util.{Cat, Decoupled, Mux1H, MuxCase, log2Ceil}
import nf_arm_doce._
import utils._

//compressed multiple sparse 512 bit axis data to 1 axis data
class axis_data_collector(AXIS_DATA_WIDTH : Int = 64) extends Module   {
  val io = IO(new Bundle() {
    val in = Flipped(Decoupled(new axisdata(AXIS_DATA_WIDTH, 4)))
    val out = Decoupled(new axisdata(AXIS_DATA_WIDTH, 4))

    val flush = Input(Bool())
    val empty = Output(Bool())
  })

  val in = Module(new axis_reg_slice(AXIS_DATA_WIDTH, "collector_reg"))
  in.io.aclk := clock.asBool()
  in.io.aresetn := ~reset.asBool()
  in.io.s_axis.tvalid := io.in.valid
  in.io.s_axis.connectfrom(io.in.bits)
  io.in.ready := in.io.s_axis.tready
  val in_data = VecInit(Seq.fill(AXIS_DATA_WIDTH/4)(0.U.asTypeOf(new axisdata(4, 4))))
  val in_count = (Seq.tabulate(AXIS_DATA_WIDTH/4)(
    i => {
      Seq.tabulate(i + 1)(x => (in.io.m_axis.tkeep(AXIS_DATA_WIDTH/4-1, 0).asBools()(x) && in.io.m_axis.tvalid.asBool()).asTypeOf(UInt(8.W))).reduce(_+_)
    }
  ))
  in_data.zipWithIndex.map{
    case (d, i) => {
      print(i)
      d.tdata := MuxCase(0.U, Seq.tabulate(AXIS_DATA_WIDTH/4 - i)(
        x => ((i.U(8.W) + 1.U(8.W)) === in_count(x + i)) -> in.io.m_axis.tdata(((x + i) + 1) * 32 - 1, (x + i) * 32))
      )
      d.tkeep := MuxCase(0.U, Seq.tabulate(AXIS_DATA_WIDTH/4 - i)(
        x => ((i.U(8.W) + 1.U(8.W)) === in_count(x + i)) -> in.io.m_axis.tkeep(x + i))
      )
    }
  }

  val sorted_in = Module(new axis_reg_slice(AXIS_DATA_WIDTH, "collector_reg"))
  sorted_in.io.aclk := clock.asBool()
  sorted_in.io.aresetn := ~reset.asBool()
  sorted_in.io.s_axis.tvalid := in.io.m_axis.tvalid & in_count.last > 0.U
  sorted_in.io.s_axis.tkeep := VecInit(in_data.map(x=>x.tkeep)).asUInt()
  sorted_in.io.s_axis.tdata := VecInit(in_data.map(x=>x.tdata)).asUInt()
  in.io.m_axis.tready := sorted_in.io.s_axis.tready | in_count.last === 0.U
  val in_count_reg = RegInit(0.U(32.W))
  when((sorted_in.io.s_axis.tvalid & sorted_in.io.s_axis.tready).asBool()){
    in_count_reg := in_count.last
  }

  val mid = Module(new axis_reg_slice(AXIS_DATA_WIDTH*2, "collector_mid_reg"))
  mid.io.aclk := clock.asBool()
  mid.io.aresetn := ~reset.asBool()
  val mid_count = RegInit(0.U(32.W))
  //val total_count = mid_count + in_count.last
  val mid_data_in = VecInit(Seq.fill(AXIS_DATA_WIDTH*3/4)(0.U.asTypeOf(new axisdata(4, 4))))
  mid_data_in.zipWithIndex.map{
    case(d, i) => {
      if(i < 32){
        d.tdata := mid.io.m_axis.tdata((i + 1) * 32 - 1, i * 32)
        d.tkeep := mid.io.m_axis.tkeep(i)
      }else{
        d.tdata := 0.U
        d.tkeep := 0.U
      }
      when(i.U >= mid_count){
        when((i.U - mid_count) < 16.U){
          d.tdata := MuxCase(0.U, Seq.tabulate(AXIS_DATA_WIDTH/4)(
            x => (x.U === (i.U - mid_count)) -> sorted_in.io.m_axis.tdata((x + 1) * 32 - 1, x * 32)
          ))
          d.tkeep := MuxCase(0.U, Seq.tabulate(AXIS_DATA_WIDTH/4)(
            x => (x.U === (i.U - mid_count)) -> sorted_in.io.m_axis.tkeep(x)
          ))
        }.otherwise{
          d.tdata := 0.U
          d.tkeep := 0.U
        }
      }
    }
  }

  val out = Module(new axis_reg_slice(AXIS_DATA_WIDTH, "collector_reg"))
  out.io.aclk := clock.asBool()
  out.io.aresetn := ~reset.asBool()
  out.io.m_axis.tready := io.out.ready
  out.io.m_axis.connectto(io.out.bits, 0)
  io.out.valid := out.io.m_axis.tvalid
  out.io.s_axis.tvalid := false.B
  out.io.s_axis.tkeep := 0.U
  out.io.s_axis.tdata := 0.U
  when(mid.io.m_axis.tvalid.asBool()){
    when((mid_count >= 16.U)
    || io.flush){
      out.io.s_axis.tvalid := true.B
      out.io.s_axis.tkeep := mid.io.m_axis.tkeep(AXIS_DATA_WIDTH/4-1, 0)
      out.io.s_axis.tdata := mid.io.m_axis.tdata(AXIS_DATA_WIDTH*8-1, 0)
    }
  }
  out.io.s_axis.tlast := true.B

  //consumed by out or updated by in
  mid.io.m_axis.tready := (out.io.s_axis.tready & out.io.s_axis.tvalid) | (mid.io.s_axis.tvalid & (mid_count < 16.U))
  mid.io.s_axis.tvalid := sorted_in.io.m_axis.tvalid.asBool() |
    (out.io.s_axis.tready & out.io.s_axis.tvalid & mid_count > 16.U)
  mid.io.s_axis.tdata := 0.U
  mid.io.s_axis.tkeep := 0.U
  when(mid.io.s_axis.tvalid.asBool()){
    when(mid.io.m_axis.tvalid.asBool()){
      when(out.io.s_axis.tvalid.asBool()){
        mid.io.s_axis.tdata := VecInit(Seq.tabulate(32)(i => mid_data_in(i+16).tdata)).asUInt()
        mid.io.s_axis.tkeep := VecInit(Seq.tabulate(32)(i => mid_data_in(i+16).tkeep)).asUInt()
      }.otherwise{
        mid.io.s_axis.tdata := VecInit(mid_data_in.map(x=>x.tdata)).asUInt()
        mid.io.s_axis.tkeep := VecInit(mid_data_in.map(x=>x.tkeep)).asUInt()
      }
    }.otherwise{
      mid.io.s_axis.tdata := sorted_in.io.m_axis.tdata
      mid.io.s_axis.tkeep := sorted_in.io.m_axis.tkeep
    }
  }
  when(sorted_in.io.m_axis.tvalid.asBool() && sorted_in.io.m_axis.tready.asBool() &&
    out.io.s_axis.tvalid.asBool() && out.io.s_axis.tready.asBool()){
    when(mid_count < 16.U){
      mid_count := in_count_reg
    }.otherwise{
      mid_count := mid_count + in_count_reg - 16.U
    }
  }.elsewhen(out.io.s_axis.tvalid.asBool() && out.io.s_axis.tready.asBool()){
    when(mid_count < 16.U){
      mid_count := 0.U
    }.otherwise{
      mid_count := mid_count - 16.U
    }
  }.elsewhen(sorted_in.io.m_axis.tvalid.asBool() && sorted_in.io.m_axis.tready.asBool()){
    mid_count := mid_count + in_count_reg
  }

  sorted_in.io.m_axis.tready := mid.io.s_axis.tready
  io.empty := (in.io.m_axis.tvalid | mid.io.m_axis.tvalid | out.io.m_axis.tvalid | sorted_in.io.m_axis.tvalid) === false.B
}

class Remote_Apply(AXIS_DATA_WIDTH: Int, Local_Scatter_Num: Int, FPGA_Num: Int, Remote_ID : Int) extends Module {
  val io = IO(new Bundle() {
    val xbar_in = Flipped(Decoupled(new axisdata(AXIS_DATA_WIDTH, 4)))
    val remote_out = Decoupled(new axisdata_u(AXIS_DATA_WIDTH, log2Ceil(FPGA_Num)))

    //control path
    val recv_sync = Input(UInt(Local_Scatter_Num.W))
    val recv_sync_phase2 = Input(Bool())
    val signal = Input(Bool())
    val signal_ack = Output(Bool())
    val local_fpga_id = Input(UInt(log2Ceil(FPGA_Num).W))
    val local_unvisited_size = Input(UInt(32.W))
    val end = Output(Bool())
    val packet_size = Input(UInt(32.W))
  })

  def vid_to_remote(vid: UInt) : Bool = {
    vid(log2Ceil(FPGA_Num) - 1, 0) === Remote_ID.U && vid(log2Ceil(FPGA_Num) - 1, 0) =/= io.local_fpga_id
  }

  val filtered_keep = Wire(Vec(AXIS_DATA_WIDTH / 4, Bool()))
  filtered_keep.zipWithIndex.map{
    case(k, i) => {
      k := Mux(io.xbar_in.bits.tkeep(i),
        Mux(io.xbar_in.bits.get_ith_data(i)(31), false.B, vid_to_remote(io.xbar_in.bits.get_ith_data(i))),
        false.B)
    }
  }

  val collector = Module(new axis_data_collector(64))
  collector.io.in.valid := io.xbar_in.valid
  collector.io.in.bits.tkeep := filtered_keep.asUInt()
  collector.io.in.bits.tdata := io.xbar_in.bits.tdata
  collector.io.in.bits.tlast := true.B
  io.xbar_in.ready := collector.io.in.ready

  val vertex_in_fifo = Module(new axis_data_count_fifo(AXIS_DATA_WIDTH, "remote_apply_vid_fifo"))
  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val output_fin_phase1 = Value(0x1.U)
    val wait_sync_phase1 = Value(0x2.U)
    val output_fin_phase2 = Value(0x3.U)
    val wait_signal = Value(0x4.U)
    val flush = Value(0x5.U)
    val wait_data_flow = Value(0x6.U)
  }
  val sync_data = (Cat(1.U(1.W), io.local_fpga_id) << (32 - 1 - log2Ceil(FPGA_Num))).asUInt() +
    io.local_unvisited_size
  val sync_status = RegInit(sm.idole)
  val stall_time = RegInit(0.U(32.W))
  //stall until the local unvisited size is updated
  when(sync_status === sm.wait_data_flow){
    stall_time := stall_time + 1.U
  }.otherwise{
    stall_time := 0.U
  }
  when(sync_status === sm.idole && io.recv_sync.andR() && io.local_fpga_id =/= Remote_ID.U) {
    sync_status := sm.flush
  }.elsewhen(sync_status === sm.flush && collector.io.empty){
    sync_status := sm.output_fin_phase1
  }.elsewhen(sync_status === sm.output_fin_phase1 && vertex_in_fifo.io.s_axis.tready.asBool()){
    sync_status := sm.wait_sync_phase1
  }.elsewhen(sync_status === sm.wait_sync_phase1 && io.recv_sync_phase2){
    sync_status := sm.wait_data_flow
  }.elsewhen(sync_status === sm.wait_data_flow && stall_time === 12.U){
    sync_status := sm.output_fin_phase2
  }.elsewhen(sync_status === sm.output_fin_phase2 && vertex_in_fifo.io.s_axis.tready.asBool()){
    sync_status := sm.wait_signal
  }.elsewhen(sync_status === sm.wait_signal && io.signal){
    sync_status := sm.idole
  }
  io.signal_ack := sync_status === sm.wait_signal
  io.end := sync_status === sm.output_fin_phase2 && vertex_in_fifo.io.s_axis.tready.asBool()
  collector.io.flush := sync_status === sm.flush

  val need_to_send_sync = (sync_status === sm.output_fin_phase1) | (sync_status === sm.output_fin_phase2)
  vertex_in_fifo.io.s_axis_aclk := clock.asBool()
  vertex_in_fifo.io.s_axis_aresetn := ~reset.asBool()
  vertex_in_fifo.io.s_axis.tvalid := (collector.io.out.valid) | need_to_send_sync
  vertex_in_fifo.io.s_axis.tkeep := Mux(need_to_send_sync, 1.U, collector.io.out.bits.tkeep)
  vertex_in_fifo.io.s_axis.tlast := 1.U
  vertex_in_fifo.io.s_axis.tdata := Mux(need_to_send_sync, sync_data, collector.io.out.bits.tdata)
  collector.io.out.ready := vertex_in_fifo.io.s_axis.tready

  //vertex_in_fifo.io.m_axis.connectto(io.remote_out.bits, 0)
  val send_count = RegInit(0.U(32.W))
  io.remote_out.bits.tkeep := vertex_in_fifo.io.m_axis.tkeep
  io.remote_out.bits.tdata := vertex_in_fifo.io.m_axis.tdata
  io.remote_out.bits.tuser := Remote_ID.U
  io.remote_out.bits.tlast := send_count === (io.packet_size - 1.U) | vertex_in_fifo.io.axis_rd_data_count === 1.U
  io.remote_out.valid := false.B
  vertex_in_fifo.io.m_axis.tready := false.B
  when(io.remote_out.valid && io.remote_out.ready){
    when(send_count === (io.packet_size - 1.U)){
      send_count := 0.U
    }.otherwise{
      send_count := send_count + 1.U
    }
  }
  when(vertex_in_fifo.io.axis_rd_data_count >= io.packet_size && send_count === 0.U
  || (send_count =/= 0.U)
  || (sync_status =/= sm.idole)){
    io.remote_out.valid := vertex_in_fifo.io.m_axis.tvalid
    vertex_in_fifo.io.m_axis.tready := io.remote_out.ready
  }
}

//multiple axis port to one axi port
class axis_to_axi(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3,
                  AXIS_DATA_WIDTH: Int, Master_Num : Int) extends Module{
  val io = IO(new Bundle() {
    val xbar_in = Vec(Master_Num, Flipped(Decoupled(new axisdata_u(AXIS_DATA_WIDTH, log2Ceil(Master_Num)))))
    val remote_out = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))

    val level_base_addr = Vec(Master_Num, Input(UInt(64.W)))
  })

  assert(AXI_DATA_WIDTH == AXIS_DATA_WIDTH)

  val switch = Module(new axis_switch(AXI_DATA_WIDTH, Master_Num, "axis_to_axi_switch", log2Ceil(Master_Num)))
  switch.io.aclk := clock.asBool()
  switch.io.aresetn := ~reset.asBool()
  switch.io.s_req_suppress := 0.U
  switch.io.s_axis.tvalid := VecInit(io.xbar_in.map(x=>x.valid)).asUInt()
  switch.io.s_axis.tdata := VecInit(io.xbar_in.map(x=>x.bits.tdata)).asUInt()
  switch.io.s_axis.tkeep := VecInit(io.xbar_in.map(x=>x.bits.tkeep)).asUInt()
  switch.io.s_axis.tlast := VecInit(io.xbar_in.map(x=>x.bits.tlast)).asUInt()
  switch.io.s_axis.tuser := VecInit(io.xbar_in.map(x=>x.bits.tuser)).asUInt()
  io.xbar_in.zipWithIndex.map{
    case(in, i) => {in.ready := switch.io.s_axis.tready(i)}
  }

  val vertex_in_fifo = Module(new axis_data_user_fifo(AXIS_DATA_WIDTH, "remote_vid_user_fifo",
    log2Ceil(Master_Num)))
  vertex_in_fifo.io.s_axis_aclk := clock.asBool()
  vertex_in_fifo.io.s_axis_aresetn := ~reset.asBool()
  vertex_in_fifo.io.s_axis <> switch.io.m_axis

  //do the sending job
  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val sending  = Value(0x1.U) // i "load"  -> 000_0011
    val wait_aw = Value(0x2.U)
    val wait_w = Value(0x3.U)
  }
  val send_status = RegInit(sm.idole)
  val addr_issue = io.remote_out.aw.valid && io.remote_out.aw.ready
  val data_issue = io.remote_out.w.valid && io.remote_out.w.ready && io.remote_out.w.bits.wlast.asBool()
  when(send_status === sm.idole && vertex_in_fifo.io.m_axis.tvalid.asBool()){
    send_status := sm.sending
  }.elsewhen((send_status === sm.sending && addr_issue && data_issue)
    || (send_status === sm.wait_aw && addr_issue)
    || (send_status === sm.wait_w && data_issue)){
    send_status := sm.idole
  }.elsewhen(send_status === sm.sending && !addr_issue && data_issue){
    send_status := sm.wait_aw
  }.elsewhen(send_status === sm.sending && addr_issue && !data_issue){
    send_status := sm.wait_w
  }
  vertex_in_fifo.io.m_axis.tready := false.B
  when(send_status === sm.sending || send_status === sm.wait_w){
    vertex_in_fifo.io.m_axis.tready := io.remote_out.w.ready
  }.elsewhen(send_status === sm.wait_aw && addr_issue){
    vertex_in_fifo.io.m_axis.tready := true.B
  }

  //tie off not used channel
  io.remote_out.ar.bits.tie_off(0.U)
  io.remote_out.ar.valid := false.B
  io.remote_out.b.ready := true.B
  io.remote_out.r.ready := true.B

  io.remote_out.aw.valid := send_status === sm.sending || send_status === sm.wait_aw
  io.remote_out.aw.bits.awlen := 0.U
  io.remote_out.aw.bits.awsize := log2Ceil(AXI_DATA_WIDTH).U
  io.remote_out.aw.bits.awburst := 1.U(2.W)
  io.remote_out.aw.bits.awlock := 0.U
  io.remote_out.aw.bits.awid := 0.U
  io.remote_out.aw.bits.awaddr := Mux1H(Seq.tabulate(Master_Num){
    i => (vertex_in_fifo.io.m_axis.tuser === i.U) -> io.level_base_addr(i)
  })

  io.remote_out.w.valid := send_status === sm.sending || send_status === sm.wait_w
  io.remote_out.w.bits.wlast := vertex_in_fifo.io.m_axis.tlast
  io.remote_out.w.bits.wdata := vertex_in_fifo.io.m_axis.tdata
  io.remote_out.w.bits.wstrb := vertex_in_fifo.io.m_axis.tkeep(AXIS_DATA_WIDTH / 4 - 1, 0)

  val packet_sent = RegInit(0.U(32.W))
  dontTouch(packet_sent)
  when(io.remote_out.aw.valid && io.remote_out.aw.ready){
    packet_sent := packet_sent + 1.U
  }
}

class Remote_Scatter(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3,
                     AXIS_DATA_WIDTH : Int, FPGA_Num: Int) extends Module {
  val io = IO(new Bundle() {
    val remote_in = new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 4, AXI_SIZE_WIDTH)
    val xbar_out = Decoupled(new axisdata(AXIS_DATA_WIDTH, 4))

    //control path
    val signal = Input(Bool())
    val start = Input(Bool())
    val issue_sync = Output(Bool())
    val issue_sync_phase2 = Output(Vec(FPGA_Num, Bool()))
    val remote_unvisited_size = Output(UInt(32.W))
    val local_fpga_id = Input(UInt(log2Ceil(FPGA_Num).W))
  })

  def get_fpga_id(vid : UInt) : UInt = {
    vid(30, 31 - log2Ceil(FPGA_Num))
  }

  def get_fpga_size(vid : UInt) : UInt = {
    vid(30 - log2Ceil(FPGA_Num), 0)
  }

  //tie off not used channel
  io.remote_in.ar.ready := false.B
  io.remote_in.aw.ready := true.B
  io.remote_in.r.bits.tie_off()
  io.remote_in.r.valid := false.B

  val vertex_in_fifo = Module(new axis_data_fifo(AXIS_DATA_WIDTH, "remote_vid_fifo"))
  vertex_in_fifo.io.s_axis_aclk := clock.asBool()
  vertex_in_fifo.io.s_axis_aresetn := ~reset.asBool()
  vertex_in_fifo.io.s_axis.tvalid := io.remote_in.w.valid
  vertex_in_fifo.io.s_axis.tkeep := io.remote_in.w.bits.wstrb
  vertex_in_fifo.io.s_axis.tdata := io.remote_in.w.bits.wdata
  io.remote_in.w.ready := vertex_in_fifo.io.s_axis.tready

  val syncs = RegInit(VecInit(Seq.fill(FPGA_Num)(0.U(FPGA_Num.W))))
  val is_sync = vertex_in_fifo.io.m_axis.tvalid.asBool() && vertex_in_fifo.io.m_axis.tdata(31) === 1.U &&
    vertex_in_fifo.io.m_axis.tkeep(0) === 1.U && !io.issue_sync
  syncs.zipWithIndex.map{
    case(sync, i) => {
      when(io.local_fpga_id === i.U){
        sync := 2.U
      }.elsewhen(is_sync && get_fpga_id(vertex_in_fifo.io.m_axis.tdata) === i.U){
        sync := sync + 1.U
      }.elsewhen(!io.start && io.signal){
        sync := 0.U
      }
    }
  }
  io.issue_sync_phase2 := VecInit(Seq.fill(FPGA_Num)(syncs.map(x=>(x > 0.U)).reduce(_&_)))
  io.issue_sync := syncs.map(x=>(x === 2.U)).reduce(_&_)
  vertex_in_fifo.io.m_axis.connectto(io.xbar_out.bits, 0)
  vertex_in_fifo.io.m_axis.tready := MuxCase(io.xbar_out.ready, Array(
    io.issue_sync -> false.B,
    is_sync -> true.B
  ))
  io.xbar_out.valid := MuxCase(vertex_in_fifo.io.m_axis.tvalid, Array(
    io.issue_sync -> false.B,
    is_sync -> false.B
  ))

  val unvisited_size_reg = RegInit(0.U(32.W))
  when(is_sync){
    unvisited_size_reg := unvisited_size_reg + get_fpga_size(vertex_in_fifo.io.m_axis.tdata)
  }.elsewhen(!io.start && io.signal){
    unvisited_size_reg := 0.U
  }
  io.remote_unvisited_size := unvisited_size_reg

  io.remote_in.b.bits.tie_off()
  io.remote_in.b.valid := false.B

  val packet_recv = RegInit(0.U(32.W))
  dontTouch(packet_recv)
  when(io.remote_in.w.valid && io.remote_in.w.ready){
    packet_recv := packet_recv + 1.U
  }

  val ready_counter = RegInit(0.U(32.W))
  dontTouch(ready_counter)
  when(io.remote_in.w.valid && io.remote_in.w.ready === false.B){
    ready_counter := ready_counter + 1.U
  }
}

class Remote_xbar(AXIS_DATA_WIDTH: Int, SLAVE_NUM: Int, MASTER_NUM: Int, FPGA_Num : Int) extends Module {
  val io = IO(new Bundle {
    val ddr_in = Vec(MASTER_NUM, Flipped(Decoupled(new axisdata(AXIS_DATA_WIDTH, 4))))
    val pe_out = Vec(SLAVE_NUM, Decoupled(new axisdata(AXIS_DATA_WIDTH * MASTER_NUM + 64, 4)))
    val remote_out = Vec(FPGA_Num, Decoupled(new axisdata(64, 4)))
    val remote_in = Flipped(Decoupled(new axisdata(64, 4)))
  })

  assert(AXIS_DATA_WIDTH * MASTER_NUM == 64)

  val combiner_level0 = Module(new axis_combiner(AXIS_DATA_WIDTH, MASTER_NUM, "axis_combiner_level0"))
  combiner_level0.io.aclk := clock.asBool()
  combiner_level0.io.aresetn := ~reset.asBool()
  combiner_level0.io.s_axis.tdata := VecInit.tabulate(MASTER_NUM){i => io.ddr_in(i).bits.tdata}.asUInt()
  combiner_level0.io.s_axis.tkeep := VecInit.tabulate(MASTER_NUM){i => VecInit(io.ddr_in(i).bits.tkeep.asBools().map{x => x &
    io.ddr_in(i).valid})}.asUInt()
  combiner_level0.io.s_axis.tlast := VecInit.tabulate(MASTER_NUM){i => io.ddr_in(i).bits.tlast}.asUInt()
  combiner_level0.io.s_axis.tvalid := VecInit(Seq.fill(MASTER_NUM)(io.ddr_in.map{i => i.valid}.reduce(_|_))).asUInt()
  io.ddr_in.zipWithIndex.map{
    case(in, i) => {in.ready := combiner_level0.io.s_axis.tready(i)}
  }

  val buffer0 = Module(new axis_data_fifo(AXIS_DATA_WIDTH * MASTER_NUM, "Remote_xbar_buffer0"))
  buffer0.io.s_axis_aclk := clock.asBool()
  buffer0.io.s_axis_aresetn := ~reset.asBool()
  buffer0.io.s_axis <> combiner_level0.io.m_axis

  val xbar_level0 = Module(new axis_broadcaster(64, 1 + FPGA_Num, "axis_broadcaster_level0"))
  xbar_level0.io.aclk := clock.asBool()
  xbar_level0.io.aresetn := ~reset.asBool()
  xbar_level0.io.s_axis.tdata := buffer0.io.m_axis.tdata
  xbar_level0.io.s_axis.tvalid := buffer0.io.m_axis.tvalid
  xbar_level0.io.s_axis.tkeep := VecInit(buffer0.io.m_axis.tkeep.asBools().map{x => x & buffer0.io.m_axis.tvalid}).asUInt()
  buffer0.io.m_axis.tready := xbar_level0.io.s_axis.tready
  io.remote_out.zipWithIndex.map{
    case (r, i) => {
      xbar_level0.io.m_axis.connectto(r.bits, i+1)
      r.valid := xbar_level0.io.m_axis.tvalid(i+1)
    }
  }

  val combiner_level1 = Module(new axis_combiner(64, 2, "axis_combiner_level1"))
  combiner_level1.io.aclk := clock.asBool()
  combiner_level1.io.aresetn := ~reset.asBool()
  combiner_level1.io.s_axis.tdata := Cat(xbar_level0.io.m_axis.tdata(511, 0), io.remote_in.bits.tdata)
  combiner_level1.io.s_axis.tkeep := Cat(xbar_level0.io.m_axis.tkeep(63, 0),
    VecInit(io.remote_in.bits.tkeep.asBools().map{x => x & io.remote_in.valid}).asUInt())
  combiner_level1.io.s_axis.tlast := 3.U
  combiner_level1.io.s_axis.tvalid := VecInit(Seq.fill(2)(xbar_level0.io.m_axis.tvalid(0) | io.remote_in.valid)).asUInt()
  io.remote_in.ready := combiner_level1.io.s_axis.tready(0)
  xbar_level0.io.m_axis.tready := Cat(VecInit(io.remote_out.map(x=>x.ready)).asUInt(), combiner_level1.io.s_axis.tready(1))

  val buffer1 = Module(new axis_data_fifo(64*2, "Remote_xbar_buffer1"))
  buffer1.io.s_axis_aclk := clock.asBool()
  buffer1.io.s_axis_aresetn := ~reset.asBool()
  buffer1.io.s_axis <> combiner_level1.io.m_axis

  val xbar_level1 = Module(new axis_broadcaster(128, SLAVE_NUM, "axis_broadcaster_level1"))
  xbar_level1.io.aclk := clock.asBool()
  xbar_level1.io.aresetn := ~reset.asBool()
  xbar_level1.io.s_axis.tdata := buffer1.io.m_axis.tdata
  xbar_level1.io.s_axis.tvalid := buffer1.io.m_axis.tvalid
  xbar_level1.io.s_axis.tkeep := VecInit(buffer1.io.m_axis.tkeep.asBools().map{x => x & buffer1.io.m_axis.tvalid}).asUInt()
  buffer1.io.m_axis.tready := xbar_level1.io.s_axis.tready
  io.pe_out.zipWithIndex.map{
    case(pe, i) => {
      xbar_level1.io.m_axis.connectto(pe.bits, i)
      pe.valid := xbar_level1.io.m_axis.tvalid(i)
    }
  }
  xbar_level1.io.m_axis.tready := (VecInit.tabulate(SLAVE_NUM)(
    i => io.pe_out(i).ready
  )).asUInt()
}