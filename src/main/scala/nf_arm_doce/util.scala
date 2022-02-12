package utils
import chisel3._
import chisel3.experimental.ChiselEnum
import chisel3.util._
import nf_arm_doce._

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
  when(io.dout.ready){
    data := io.din.bits
    valid := io.din.valid
  }

  io.din.ready := io.dout.ready
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

private object ArbiterCtrl {
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
  val grant = ArbiterCtrl(io.in.map(_.valid))
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

  when(io.in.map(_.valid).reduce(_|_) && status === sm.idole){
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