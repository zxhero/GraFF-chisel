package remote

import chisel3._
import chisel3.experimental.ChiselEnum
import chisel3.util.{Arbiter, Cat, Decoupled, Mux1H, MuxCase, log2Ceil}
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
        when(mid_count < 16.U){
          mid.io.s_axis.tdata := sorted_in.io.m_axis.tdata
          mid.io.s_axis.tkeep := sorted_in.io.m_axis.tkeep
        }.otherwise{
          mid.io.s_axis.tdata := VecInit(Seq.tabulate(32)(i => mid_data_in(i+16).tdata)).asUInt()
          mid.io.s_axis.tkeep := VecInit(Seq.tabulate(32)(i => mid_data_in(i+16).tkeep)).asUInt()
        }
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

class sync_msg (FPGA_Num : Int) extends Bundle() {
  val flag = Bool()
  val id = UInt(log2Ceil(FPGA_Num).W)
  val level = UInt(4.W)
  val reserved = UInt((32 - 1 - log2Ceil(FPGA_Num) - 4 - 24).W)
  val size = UInt(24.W)
}

class flow_control(FPGA_Num: Int) extends Module{
  val io = IO(new Bundle() {
    val data = Input(UInt(512.W))
    val keep = Input(UInt(16.W))
    val pending = Output(UInt(32.W))
  })

  val hittable = Wire(Vec(16, Vec(16, Bool())))
  dontTouch(hittable)
  io.data.asTypeOf(Vec(16, UInt(32.W))).zipWithIndex.map{
    case(d, x) => {
      for (i <- 15 to 0 by -1){
        when(i.U === d(log2Ceil(16) + log2Ceil(FPGA_Num) - 1, log2Ceil(FPGA_Num))){
          hittable(x)(i) := io.keep(x)
        }.otherwise{
          hittable(x)(i) := false.B
        }
      }
    }
  }
  val count = VecInit(Seq.tabulate(16)(x => hittable.map(h => h(x).asTypeOf(UInt(5.W))).reduce(_+_)))
  dontTouch(count)
  when(count.do_exists(_ >= 2.U)){
    io.pending := (2*FPGA_Num).U
  }.otherwise{
    io.pending := FPGA_Num.U
  }
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
    val level = Input(UInt(32.W))
    val pending_time = Input(UInt(32.W))
  })

  val level = RegInit(0.U(32.W))
  level := io.level

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
  val sync_data = Wire(new sync_msg(FPGA_Num))
  sync_data.flag := (true.B)
  sync_data.id := io.local_fpga_id
  sync_data.level := level
  sync_data.reserved := 0.U
  sync_data.size := io.local_unvisited_size(23, 0)
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
  }.elsewhen(sync_status === sm.wait_data_flow && stall_time === 30.U){
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
  vertex_in_fifo.io.s_axis.tdata := Mux(need_to_send_sync, sync_data.asUInt(), collector.io.out.bits.tdata)
  collector.io.out.ready := vertex_in_fifo.io.s_axis.tready

  //vertex_in_fifo.io.m_axis.connectto(io.remote_out.bits, 0)
  val send_count = RegInit(0.U(32.W))
  val pending_time = RegInit(0.U(32.W))
  val extra_time = RegInit(0.U(32.W))
  val next_pending = RegInit(0.U(32.W))
  val flow_control_unit = Module(new flow_control(FPGA_Num))
  io.remote_out.bits.tkeep := vertex_in_fifo.io.m_axis.tkeep
  io.remote_out.bits.tdata := vertex_in_fifo.io.m_axis.tdata
  io.remote_out.bits.tuser := Remote_ID.U
  io.remote_out.bits.tlast := send_count === (io.packet_size - 1.U) | vertex_in_fifo.io.axis_rd_data_count === 1.U
  flow_control_unit.io.data := vertex_in_fifo.io.m_axis.tdata
  flow_control_unit.io.keep := vertex_in_fifo.io.m_axis.tkeep
  when(io.remote_out.valid && io.remote_out.ready){
    when(io.remote_out.bits.tlast){
      next_pending := 0.U
    }.otherwise{
      next_pending := flow_control_unit.io.pending
    }
  }
  when(io.remote_out.valid && io.remote_out.ready && io.remote_out.bits.tlast){
    extra_time := 0.U
  }.elsewhen(pending_time === 0.U){
    extra_time := extra_time + 1.U
  }
  io.remote_out.valid := false.B
  vertex_in_fifo.io.m_axis.tready := false.B
  when(io.remote_out.valid && io.remote_out.ready && io.remote_out.bits.tlast){
    pending_time := Mux(extra_time > next_pending + flow_control_unit.io.pending - 1.U,
      0.U, next_pending + flow_control_unit.io.pending - 1.U - extra_time)
  }.elsewhen(pending_time =/= 0.U){
    pending_time := pending_time - 1.U
  }
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
    when(pending_time === 0.U){
      io.remote_out.valid := vertex_in_fifo.io.m_axis.tvalid
      vertex_in_fifo.io.m_axis.tready := io.remote_out.ready
    }
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

  //tie off not used channel
  io.remote_in.ar.ready := false.B
  io.remote_in.aw.ready := true.B
  io.remote_in.r.bits.tie_off()
  io.remote_in.r.valid := false.B

  val vertex_in_fifo = Module(new axis_data_uram_fifo(AXIS_DATA_WIDTH, "remote_vid_fifo"))
  vertex_in_fifo.io.s_axis_aclk := clock.asBool()
  vertex_in_fifo.io.s_axis_aresetn := ~reset.asBool()
  vertex_in_fifo.io.s_axis.tvalid := io.remote_in.w.valid
  vertex_in_fifo.io.s_axis.tkeep := io.remote_in.w.bits.wstrb
  vertex_in_fifo.io.s_axis.tdata := io.remote_in.w.bits.wdata
  io.remote_in.w.ready := vertex_in_fifo.io.s_axis.tready

  val sync_data = vertex_in_fifo.io.m_axis.tdata(31, 0).asTypeOf(new sync_msg(FPGA_Num))
  val syncs = RegInit(VecInit(Seq.fill(FPGA_Num)(0.U(FPGA_Num.W))))
  val is_sync = vertex_in_fifo.io.m_axis.tvalid.asBool() && sync_data.flag &&
    vertex_in_fifo.io.m_axis.tkeep(0) === 1.U && !io.issue_sync
  syncs.zipWithIndex.map{
    case(sync, i) => {
      when(io.local_fpga_id === i.U && io.start){
        sync := 2.U
      }.elsewhen(is_sync && sync_data.id === i.U){
        sync := sync + 1.U
      }.elsewhen(!io.start && io.signal && io.local_fpga_id =/= i.U){
        sync := 0.U
      }
    }
  }
  io.issue_sync_phase2 := VecInit(Seq.fill(FPGA_Num)(syncs.map(x=>(x > 0.U)).reduce(_&_)))
  io.issue_sync := syncs.map(x=>(x === 2.U)).reduce(_&_)
  vertex_in_fifo.io.m_axis.connectto(io.xbar_out.bits, 0)
  io.xbar_out.bits.tlast := true.B
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
    unvisited_size_reg := unvisited_size_reg + sync_data.size
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
    val pe_out = Vec(SLAVE_NUM, Decoupled(new axisdata(64, 4)))
    val remote_out = Vec(FPGA_Num, Decoupled(new axisdata(64, 4)))
    val remote_in = Flipped(Decoupled(new axisdata(64, 4)))

    val local_fpga_id = Input(UInt(log2Ceil(FPGA_Num).W))
    val flush = Input(Bool())
    val signal = Input(Bool())
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

  val frontend = Module(new axis_reg_slice(64, "Remote_xbar_reg_slice"))
  frontend.io.aclk := clock.asBool()
  frontend.io.aresetn := ~reset.asBool()
  frontend.io.s_axis.tvalid := combiner_level0.io.m_axis.tvalid
  frontend.io.s_axis.tdata := combiner_level0.io.m_axis.tdata
  frontend.io.s_axis.tkeep := combiner_level0.io.m_axis.tkeep
  frontend.io.s_axis.tlast := combiner_level0.io.m_axis.tlast
  combiner_level0.io.m_axis.tready := frontend.io.s_axis.tready

  val xbar_level0 = Module(new axis_broadcaster(64, 1 + FPGA_Num, "axis_broadcaster_level0"))
  xbar_level0.io.aclk := clock.asBool()
  xbar_level0.io.aresetn := ~reset.asBool()
  xbar_level0.io.s_axis.tdata := frontend.io.m_axis.tdata
  xbar_level0.io.s_axis.tvalid := frontend.io.m_axis.tvalid
  xbar_level0.io.s_axis.tkeep := VecInit(frontend.io.m_axis.tkeep.asBools().map{x => x & frontend.io.m_axis.tvalid}).asUInt()
  frontend.io.m_axis.tready := xbar_level0.io.s_axis.tready
  io.remote_out.zipWithIndex.map{
    case (r, i) => {
      xbar_level0.io.m_axis.connectto(r.bits, i+1)
      r.valid := xbar_level0.io.m_axis.tvalid(i+1)
    }
  }

  def vid_to_local(vid: UInt) : Bool = {
    vid(log2Ceil(FPGA_Num) - 1, 0) === io.local_fpga_id
  }

  val filtered_keep = Wire(Vec(64 / 4, Bool()))
  filtered_keep.zipWithIndex.map{
    case(k, i) => {
      k := Mux(xbar_level0.io.m_axis.tkeep(i),
        Mux(xbar_level0.io.m_axis.tdata(i*32 + 31), true.B, vid_to_local(xbar_level0.io.m_axis.tdata(i*32+31, i*32))),
        false.B)
    }
  }

  val flush_reg = RegInit(false.B)
  when(io.signal){
    flush_reg := false.B
  }.elsewhen(io.flush){
    flush_reg := true.B
  }
  val collector = Module(new axis_data_collector(64))
  collector.io.in.valid := xbar_level0.io.m_axis.tvalid(0)
  collector.io.in.bits.tkeep := filtered_keep.asUInt()
  collector.io.in.bits.tdata := xbar_level0.io.m_axis.tdata(511, 0)
  collector.io.in.bits.tlast := true.B
  xbar_level0.io.m_axis.tready := Cat(VecInit(io.remote_out.map(x=>x.ready)).asUInt(), collector.io.in.ready)
  collector.io.flush := flush_reg

  val buffer1 = Module(new axis_data_fifo(64, "Remote_xbar_buffer1"))
  buffer1.io.s_axis_aclk := clock.asBool()
  buffer1.io.s_axis_aresetn := ~reset.asBool()
  buffer1.io.s_axis.connectfrom(collector.io.out.bits)
  buffer1.io.s_axis.tvalid := collector.io.out.valid
  collector.io.out.ready := buffer1.io.s_axis.tready

  val remote_in_reg = Module(new axis_reg_slice(64, "Remote_xbar_reg_slice"))
  remote_in_reg.io.aclk := clock.asBool()
  remote_in_reg.io.aresetn := ~reset.asBool()
  remote_in_reg.io.s_axis.connectfrom(io.remote_in.bits)
  io.remote_in.ready := remote_in_reg.io.s_axis.tready
  remote_in_reg.io.s_axis.tvalid := io.remote_in.valid

  val switch_level1 = Module(new Arbiter(new axisdata(64), 2))
  buffer1.io.m_axis.connectto(switch_level1.io.in(1).bits, 0)
  switch_level1.io.in(1).valid := buffer1.io.m_axis.tvalid
  buffer1.io.m_axis.tready := switch_level1.io.in(1).ready
  remote_in_reg.io.m_axis.connectto(switch_level1.io.in(0).bits, 0)
  switch_level1.io.in(0).valid := remote_in_reg.io.m_axis.tvalid
  remote_in_reg.io.m_axis.tready := switch_level1.io.in(0).ready

  //filter out replication data
  val in_data = switch_level1.io.out.bits.tdata.asTypeOf(Vec(16, UInt((32).W)))
  val in_keep = VecInit(switch_level1.io.out.bits.tkeep(15, 0).asBools())
  val replication = VecInit(Seq.fill(16)(false.B))
  replication.tail.zipWithIndex.map{
    case(r, i) => {
      print(i)
      r := in_data.zip(in_keep).slice(0, i+1).map {
        case (x, k) => (x === in_data(i + 1)) & k
      }.reduce(_|_)
    }
  }
  val mid = Module(new axis_reg_slice(64, "Remote_xbar_reg_slice"))
  mid.io.aclk := clock.asBool()
  mid.io.aresetn := ~reset.asBool()
  mid.io.s_axis.tvalid := switch_level1.io.out.valid
  mid.io.s_axis.tdata := switch_level1.io.out.bits.tdata
  mid.io.s_axis.tlast := switch_level1.io.out.bits.tlast
  mid.io.s_axis.tkeep := switch_level1.io.out.bits.tkeep & VecInit(replication.map(!_)).asUInt()
  switch_level1.io.out.ready := mid.io.s_axis.tready

  val xbar_level1 = Module(new axis_broadcaster(64, SLAVE_NUM, "axis_broadcaster_level1"))
  xbar_level1.io.aclk := clock.asBool()
  xbar_level1.io.aresetn := ~reset.asBool()
  xbar_level1.io.s_axis.tdata := mid.io.m_axis.tdata
  xbar_level1.io.s_axis.tvalid := mid.io.m_axis.tvalid
  xbar_level1.io.s_axis.tkeep := VecInit(mid.io.m_axis.tkeep.asBools().map{x => x & mid.io.m_axis.tvalid}).asUInt()
  mid.io.m_axis.tready := xbar_level1.io.s_axis.tready

  val backend = Seq.fill(SLAVE_NUM)(Module(new axis_reg_slice(64, "Remote_xbar_reg_slice")))
  backend.zipWithIndex.map{
    case(b, i) => {
      b.io.aclk := clock.asBool()
      b.io.aresetn := !reset.asBool()
      b.io.s_axis.tdata := xbar_level1.io.m_axis.tdata(512 * (i + 1) - 1, 512 * i)
      b.io.s_axis.tkeep := xbar_level1.io.m_axis.tkeep(64 * (i + 1) - 1, 64 * i)
      b.io.s_axis.tlast := xbar_level1.io.m_axis.tlast(i)
      b.io.s_axis.tvalid := xbar_level1.io.m_axis.tvalid(i)
    }
  }
  xbar_level1.io.m_axis.tready := (VecInit.tabulate(SLAVE_NUM)(
    i => backend(i).io.s_axis.tready
  )).asUInt()
  io.pe_out.zipWithIndex.map{
    case(pe, i) => {
      backend(i).io.m_axis.connectto(pe.bits, 0)
      pe.valid := backend(i).io.m_axis.tvalid
      backend(i).io.m_axis.tready := pe.ready
    }
  }
}