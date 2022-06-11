package utils
import chisel3._
import chisel3.experimental.ChiselEnum
import chisel3.util._
import nf_arm_doce._

class LookupTable(width: Int = 32, depth : Int, AXI_ADDR_WIDTH : Int = 44) extends MultiIOModule {
  val io = IO(new Bundle() {
    val data = Output(Vec(depth, UInt(width.W)))
    val dataIn = Input(Vec(2, UInt(width.W)))
    val writeFlag = Input(Vec(2, Bool()))
    val wptr = Input(Vec(2, UInt((log2Ceil(depth)).W)))

  })
  val config = IO(new axilitedata(AXI_ADDR_WIDTH))
  val table = RegInit(VecInit(Seq.fill(depth)(0.U(width.W))))

  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val get_aw  = Value(0x1.U) // i "load"  -> 000_0011
    val get_w = Value(0x2.U)
    val w_done = Value(0x3.U)
    val get_ar = Value(0x4.U)
    val r_done = Value(0x5.U)
  }
  val status = RegInit(sm.idole)

  when(status === sm.idole){
    when(config.awvalid){
      status := sm.get_aw
    }.elsewhen(config.arvalid){
      status := sm.get_ar
    }
  }.elsewhen(status === sm.get_aw){
    when(config.wvalid){
      status := sm.get_w
    }
  }.elsewhen(status === sm.get_w){
    status := sm.w_done
  }.elsewhen(status === sm.w_done){
    when(config.bready){
      status := sm.idole
    }
  }.elsewhen(status === sm.get_ar){
    status := sm.r_done
  }.elsewhen(status === sm.r_done) {
    when(config.rready) {
      status := sm.idole
    }
  }

  //connections
  io.data := table
  config.wready := status === sm.get_aw
  config.awready := status === sm.idole
  config.arready := status === sm.idole
  config.rvalid := status === sm.r_done
  config.bvalid := status === sm.w_done
  config.bresp := (0.U(2.W))
  config.rresp := (0.U(2.W))

  //write channel
  val wbytes = config.wdata
  val wvalid = config.wvalid && config.wready
  val ewaddr = Reg(UInt((log2Ceil(depth)).W))
  when(config.awvalid && config.awready){
    ewaddr := config.awaddr((log2Ceil(depth) + log2Ceil(width / 8) - 1), log2Ceil(width / 8))
  }

  when(wvalid && io.writeFlag.reduce(_&_)){
    table(ewaddr) := wbytes.asUInt()
    table(io.wptr(0)) := io.dataIn(0)
    table(io.wptr(1)) := io.dataIn(1)
  }.elsewhen(wvalid && io.writeFlag(0)){
    table(ewaddr) := wbytes.asUInt()
    table(io.wptr(0)) := io.dataIn(0)
  }.elsewhen(wvalid && io.writeFlag(1)){
    table(ewaddr) := wbytes.asUInt()
    table(io.wptr(1)) := io.dataIn(1)
  }.elsewhen(wvalid){
    table(ewaddr) := wbytes.asUInt()
  }.elsewhen(io.writeFlag.reduce(_&_)){
    table(io.wptr(0)) := io.dataIn(0)
    table(io.wptr(1)) := io.dataIn(1)
  }.elsewhen(io.writeFlag(0)){
    table(io.wptr(0)) := io.dataIn(0)
  }.elsewhen(io.writeFlag(1)){
    table(io.wptr(1)) := io.dataIn(1)
  }

  //read address channel
  val eraddr = Reg(UInt((log2Ceil(depth)).W))
  when(config.arvalid && config.arready){
    eraddr := config.araddr((log2Ceil(depth) + log2Ceil(width / 8) - 1), log2Ceil(width / 8))
  }

  //read channel
  val rdata = table(eraddr)
  config.rdata := rdata
}

//URAM and BRAM's read latency is 1 cycle
class URAMIO(size : Int, width : Int) extends Bundle{
  val addra = Input(UInt(log2Ceil(size).W))
  val clka = Input(Bool())
  val dina = Input(UInt(width.W))
  val douta = Output(UInt(width.W))
  val ena = Input(Bool())
  val wea = Input(Bool())
  val addrb = Input(UInt(log2Ceil(size).W))
  val clkb = Input(Bool())
  val dinb = Input(UInt(width.W))
  val doutb = Output(UInt(width.W))
  val enb = Input(Bool())
  val web = Input(Bool())
}
class URAM(size : Int, width : Int) extends BlackBox{
  val io = IO(new URAMIO(size, width))
}

class URAM_cluster(size : Int, width : Int) extends Module{
  val io = IO(new URAMIO(size, width))
  val cluster = Seq.fill(size / 4096)(Module(new URAM(4096, width)))
  val doutb = Wire(Vec(size / 4086, UInt(width.W)))
  val douta = Wire(Vec(size / 4086, UInt(width.W)))

  cluster.zipWithIndex.map{
    case (u, i) => {
      u.io.clkb := io.clkb
      u.io.clka := io.clka
      u.io.enb := io.enb
      u.io.ena := io.ena
      u.io.dinb := io.dinb
      u.io.dina := io.dina
      u.io.addrb := io.addrb(11, 0)
      u.io.addra := io.addra(11, 0)
      u.io.web := io.web && (i.asUInt() === (io.addrb >> 12.U).asUInt())
      u.io.wea := io.wea && (i.asUInt() === (io.addra >> 12.U).asUInt())
      doutb(i) := Mux((i.asUInt() === (io.addrb >> 12.U).asUInt()), u.io.doutb, 0.U(width.W))
      douta(i) := Mux((i.asUInt() === (io.addra >> 12.U).asUInt()), u.io.douta, 0.U(width.W))
    }
  }

  io.doutb := doutb.reduce(_|_)
  io.douta := douta.reduce(_|_)
}

//TODO: use 9bit instead of 1bit 9 * 116509
class BRAM(size : Int = 1024 * 1024, width : Int = 1, val mname : String) extends BlackBox{
  val io = IO(new Bundle() {
    val addra = Input(UInt(log2Ceil(size).W))
    val clka = Input(Bool())
    val dina = Input(UInt(width.W))
    //val douta = Input(UInt(width.W))
    val ena = Input(Bool())
    val wea = Input(Bool())
    val addrb = Input(UInt(log2Ceil(size).W))
    val clkb = Input(Bool())
    //val dinb = Input(UInt(width.W))
    val doutb = Output(UInt(width.W))
    val enb = Input(Bool())
    //val web = Input(Bool())
  })

  override def desiredName = mname
}

class regFile (val size : Int, val width : Int) extends Module {
  val io = IO(new Bundle {
    val dataIn = Input(UInt(width.W))
    val dataOut = Output(UInt(width.W))
    val writeFlag = Input(Bool())
    val rptr = Input(UInt((log2Ceil(size)).W))
    val wptr = Input(UInt((log2Ceil(size)).W))
    val wcount = Output(UInt(log2Ceil(size).W))
  })

  val regs = RegInit(VecInit(Seq.fill(size)(0.U(width.W))))

  io.dataOut := regs(io.rptr)

  when(io.writeFlag){
    regs(io.wptr) := io.dataIn
  }

  val (counterValue, counterWrap) = Counter(io.writeFlag, size)
  io.wcount := counterValue
}

class BRAM_fifo(val size : Int, val width : Int, val mname : String) extends BlackBox{
  val io = IO(new Bundle() {
    val full = Output(Bool())
    val din = Input(UInt(width.W))
    val wr_en = Input(Bool())
    val empty = Output(Bool())
    val dout = Output(UInt(width.W))
    val rd_en = Input(Bool())
    val data_count = Output(UInt((log2Ceil(size) + 1).W))
    val clk = Input(Bool())
    val srst = Input(Bool())
    val valid = Output(Bool())
  })

  override def desiredName = mname

  def test_FIN() : Bool = {
    io.dout((width - 1).U) === 1.U && io.empty === false.B
  }

  def is_empty() : Bool = {
    io.data_count === 0.U
  }

  def is_valid() : Bool = {
    io.valid
  }

  def is_ready() : Bool = {
    io.full === false.B
  }
}

class xbar(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 8, AXI_ID_WIDTH: Int = 1, AXI_SIZE_WIDTH: Int = 3) extends BlackBox {
  val io = IO(new Bundle {
    val s_axi = new axidata_blackbox(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 2)
    val m_axi = Flipped(new axidata_blackbox(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val aclk = Input(Bool())
    val aresetn = Input(Bool())
  })
}

class pipeline[T <: Data](gen: T) extends Module{
  val io = IO(new Bundle() {
    val dout = Decoupled(gen)
    val din = Flipped(Decoupled(gen))
  })

  val data = RegInit(0.U.asTypeOf(gen))
  val valid = RegInit(false.B)
  val ready = RegInit(false.B)
  val data2 = RegInit(0.U.asTypeOf((gen)))
  val valid2 = RegInit(false.B)
  when(io.dout.ready){
    when(valid2){
      data := data2
      valid := valid2
    }.elsewhen(ready){
      data := io.din.bits
      valid := io.din.valid
    }.otherwise{
      data := 0.U.asTypeOf((gen))
      valid := false.B
    }
  }
  ready := io.dout.ready

  //din produce 1 without consumer
  when(!io.dout.ready && ready ){
    data2 := io.din.bits
    valid2 := io.din.valid
  }.elsewhen(io.dout.ready){
    //consumer takes 1
    data2 := 0.U.asTypeOf((gen))
    valid2 := false.B
  }

  io.din.ready := ready
  io.dout.valid := valid
  io.dout.bits := data
}

class axis_broadcaster (AXIS_DATA_WIDTH: Int = 64, NUM : Int, val mname : String, AXIS_ID_WIDTH: Int = 1) extends BlackBox{
  val io = IO(new Bundle() {
    val aclk = Input(Bool())
    val aresetn = Input(Bool())
    val s_axis = new streamdata_id_blackbox(AXIS_DATA_WIDTH, 1, 1, AXIS_ID_WIDTH)
    val m_axis = Flipped(new streamdata_id_blackbox(AXIS_DATA_WIDTH, 1, NUM, AXIS_ID_WIDTH))

  })

  override def desiredName = mname
}

class axis_combiner (AXIS_DATA_WIDTH: Int, NUM : Int, val mname : String, AXIS_ID_WIDTH: Int = 1) extends BlackBox{
  val io = IO(new Bundle() {
    val aclk = Input(Bool())
    val aresetn = Input(Bool())
    val s_axis = new streamdata_id_blackbox(AXIS_DATA_WIDTH, 1, NUM, AXIS_ID_WIDTH)
    val m_axis = Flipped(new streamdata_id_blackbox(AXIS_DATA_WIDTH * NUM, 1, 1, AXIS_ID_WIDTH))

  })

  override def desiredName = mname
}

class axis_data_fifo (AXIS_DATA_WIDTH: Int, val mname : String, AXIS_ID_WIDTH: Int = 1) extends BlackBox{
  val io = IO(new Bundle() {
    val s_axis_aclk = Input(Bool())
    val s_axis_aresetn = Input(Bool())
    val s_axis = new streamdata_id_blackbox(AXIS_DATA_WIDTH, 1, 1, AXIS_ID_WIDTH)
    val m_axis = Flipped(new streamdata_id_blackbox(AXIS_DATA_WIDTH, 1, 1, AXIS_ID_WIDTH))

  })

  override def desiredName = mname
}

class axis_data_count_fifo (AXIS_DATA_WIDTH: Int, val mname : String) extends BlackBox{
  val io = IO(new Bundle() {
    val s_axis_aclk = Input(Bool())
    val s_axis_aresetn = Input(Bool())
    val s_axis = new streamdata_blackbox(AXIS_DATA_WIDTH, 1, 1)
    val m_axis = Flipped(new streamdata_blackbox(AXIS_DATA_WIDTH, 1, 1))
    val axis_rd_data_count = Output(UInt(32.W))
  })

  override def desiredName = mname

  def is_empty() : Bool = {
    io.axis_rd_data_count === 0.U
  }
}

class axis_data_user_fifo (AXIS_DATA_WIDTH: Int, val mname : String, AXIS_USER_WIDTH: Int = 1) extends BlackBox{
  val io = IO(new Bundle() {
    val s_axis_aclk = Input(Bool())
    val s_axis_aresetn = Input(Bool())
    val s_axis = new streamdata_user_blackbox(AXIS_DATA_WIDTH, 1, 1, AXIS_USER_WIDTH)
    val m_axis = Flipped(new streamdata_user_blackbox(AXIS_DATA_WIDTH, 1, 1, AXIS_USER_WIDTH))
  })

  override def desiredName = mname
}

class axis_data_uram_fifo (AXIS_DATA_WIDTH: Int, val mname : String) extends BlackBox{
  val io = IO(new Bundle() {
    val s_axis_aclk = Input(Bool())
    val s_axis_aresetn = Input(Bool())
    val s_axis = new streamdata_bare_blackbox(AXIS_DATA_WIDTH, 1, 1)
    val m_axis = Flipped(new streamdata_bare_blackbox(AXIS_DATA_WIDTH, 1, 1))

  })

  override def desiredName = mname
}

class axis_switch (AXIS_DATA_WIDTH: Int, NUM : Int, val mname : String, AXIS_USER_WIDTH: Int = 1) extends BlackBox{
  val io = IO(new Bundle() {
    val aclk = Input(Bool())
    val aresetn = Input(Bool())
    val s_axis = new streamdata_user_blackbox(AXIS_DATA_WIDTH, 1, NUM, AXIS_USER_WIDTH)
    val m_axis = Flipped(new streamdata_user_blackbox(AXIS_DATA_WIDTH, 1, 1, AXIS_USER_WIDTH))
    val s_req_suppress = Input(UInt(NUM.W))
  })

  override def desiredName = mname
}

object AMBA_ArbiterCtrl {
  def apply(request: Seq[Bool]): Seq[Bool] = request.length match {
    case 0 => Seq()
    case 1 => Seq(true.B)
    case _ => true.B +: request.tail.init.scanLeft(request.head)(_ || _).map(!_)
  }
}

class AMBA_Arbiter[T <: Data](val gen: T, val n: Int) extends Module {
  val io = IO(new ArbiterIO(gen, n))

  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val fire  = Value(0x1.U) // i "load"  -> 000_0011
  }
  val status = RegInit(sm.idole)
  val grant = AMBA_ArbiterCtrl(io.in.map(_.valid))
  val grant_reg = RegInit(VecInit(Seq.fill(n)(false.B)))

  io.chosen := (n - 1).asUInt
  io.out.bits := io.in(n - 1).bits
  for (i <- n - 2 to 0 by -1) {
    when(status === sm.idole){
      when(io.in(i).valid) {
        io.chosen := i.asUInt
        io.out.bits := io.in(i).bits
      }
    }.elsewhen(status === sm.fire){
      when(grant_reg(i)) {
        io.chosen := i.asUInt
        io.out.bits := io.in(i).bits
      }
    }
  }

  when(io.in.map(_.valid).reduce(_|_) && status === sm.idole && io.out.ready === false.B){
    grant_reg := grant.zip(io.in).map{
      case(g, in) => g & in.valid
    }
    status := sm.fire
  }.elsewhen(io.out.valid && io.out.ready && status === sm.fire) {
    status := sm.idole
  }
  for (((in, i)) <- io.in.zipWithIndex) {
    in.ready := Mux(status === sm.idole, grant(i) && io.out.ready, grant_reg(i) && io.out.ready)
  }
  io.out.valid := Mux(status === sm.idole, !grant.last || io.in.last.valid, !grant_reg.last || io.in.last.valid)

}

class axis_reg_slice(AXIS_DATA_WIDTH: Int, val mname : String) extends BlackBox{
  val io = IO(new Bundle() {
    val aclk = Input(Bool())
    val aresetn = Input(Bool())
    val s_axis = new streamdata_blackbox(AXIS_DATA_WIDTH, 1, 1)
    val m_axis = Flipped(new streamdata_blackbox(AXIS_DATA_WIDTH, 1, 1))

  })

  override def desiredName = mname
}

class axis_user_reg_slice(AXIS_DATA_WIDTH: Int, AXIS_USER_WIDTH: Int, val mname : String) extends BlackBox{
  val io = IO(new Bundle() {
    val aclk = Input(Bool())
    val aresetn = Input(Bool())
    val s_axis = new streamdata_user_blackbox(AXIS_DATA_WIDTH, 1, 1, AXIS_USER_WIDTH)
    val m_axis = Flipped(new streamdata_user_blackbox(AXIS_DATA_WIDTH, 1, 1, AXIS_USER_WIDTH))

  })

  override def desiredName = mname
}

class sample_max(MaxValue : Int) extends Module{
  val io = IO(new Bundle() {
    val enable = Input(Bool())
    val clear = Input(Bool())
  })

  val (a, b) = Counter(Range(0, MaxValue), io.enable, io.clear)

  val max = RegInit(0.U(32.W))
  dontTouch(max)
  when(a > max){
    max := a
  }
}

class axis_arbitrator(AXIS_DATA_WIDTH: Int = 4, NUM : Int, ELEMENT_WIDTH: Int = 4) extends Module{
  val io = IO(new Bundle() {
    val xbar_in = Flipped(Decoupled(new axisdata(AXIS_DATA_WIDTH * NUM, ELEMENT_WIDTH)))
    val ddr_out = Decoupled(new axisdata(AXIS_DATA_WIDTH, ELEMENT_WIDTH))
  })
  assert(AXIS_DATA_WIDTH == ELEMENT_WIDTH)
  assert(NUM + log2Ceil(NUM) + 1 < AXIS_DATA_WIDTH * NUM)
  val in = Module(new axis_reg_slice(AXIS_DATA_WIDTH * NUM,
    "axis_arbitrator_in_reg_slice_"+(AXIS_DATA_WIDTH * NUM).toString))
  in.io.aclk := clock.asBool()
  in.io.aresetn := ~reset.asBool()
  in.io.s_axis.connectfrom(io.xbar_in.bits)
  in.io.s_axis.tvalid := io.xbar_in.valid
  io.xbar_in.ready := in.io.s_axis.tready

  val in_keep = VecInit(in.io.m_axis.tkeep(AXIS_DATA_WIDTH / ELEMENT_WIDTH * NUM - 1, 0).asBools())
  val in_count = in_keep.map(x=>x.asTypeOf(UInt((log2Ceil(NUM) + 1).W))).reduce(_+_)
  val mid = Module(new axis_reg_slice(AXIS_DATA_WIDTH * NUM,
    "axis_arbitrator_in_reg_slice_"+(AXIS_DATA_WIDTH * NUM).toString))
  mid.io.aclk := clock.asBool()
  mid.io.aresetn := ~reset.asBool()
  mid.io.s_axis.tvalid := in.io.m_axis.tvalid
  mid.io.s_axis.tdata := in.io.m_axis.tdata
  mid.io.s_axis.tlast := in.io.m_axis.tlast
  mid.io.s_axis.tkeep := Cat(in_count, in.io.m_axis.tkeep(AXIS_DATA_WIDTH * NUM - log2Ceil(NUM) - 2, 0))
  in.io.m_axis.tready := mid.io.s_axis.tready

  val data = mid.io.m_axis.tdata
  val keep = VecInit(mid.io.m_axis.tkeep(AXIS_DATA_WIDTH / ELEMENT_WIDTH * NUM - 1, 0).asBools())
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
      when(mid.io.m_axis.tready.asBool() && mid.io.m_axis.tvalid.asBool()){
        a := false.B
      }.elsewhen(out.io.s_axis.tvalid.asBool() && out.io.s_axis.tready.asBool()){
        a := a | b
      }
    }
  }

  val count = RegInit(0.U((log2Ceil(NUM) + 1).W))
  val next_count = mid.io.m_axis.tkeep(AXIS_DATA_WIDTH * NUM - 1, AXIS_DATA_WIDTH * NUM - log2Ceil(NUM) - 1)
  when(out.io.s_axis.tvalid.asBool() && out.io.s_axis.tready.asBool()){
    when(count === 0.U){
      count := next_count - 1.U
    }.otherwise{
      count := count - 1.U
    }
  }
  mid.io.m_axis.tready := out.io.s_axis.tready.asBool() && (count === 1.U || next_count === 1.U)

  out.io.s_axis.tvalid := choosen_keep.reduce(_|_) && mid.io.m_axis.tvalid.asBool()
  out.io.s_axis.tkeep := 1.U
  out.io.s_axis.tlast := true.B
  out.io.s_axis.tdata :=
    Mux1H(Seq.tabulate(NUM)(x => (choosen_keep(x) -> data((x + 1) * AXIS_DATA_WIDTH * 8 - 1, x * AXIS_DATA_WIDTH * 8))))
  out.io.m_axis.connectto(io.ddr_out.bits, 0)
  io.ddr_out.valid := out.io.m_axis.tvalid
  out.io.m_axis.tready := io.ddr_out.ready
}