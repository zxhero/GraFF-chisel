package utils
import chisel3._
import chisel3.util._
import nf_arm_doce._

//URAM and BRAM's read latency is 1 cycle
class URAM(size : Int = 1024, width : Int = 32) extends BlackBox{
  val io = IO(new Bundle() {
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
  })
}

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
    val data_count = Output(UInt((log2Ceil(size)).W))
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

class axis_broadcaster (AXIS_DATA_WIDTH: Int = 64, NUM : Int) extends BlackBox{
  val io = IO(new Bundle() {
    val aclk = Input(Bool())
    val aresetn = Input(Bool())
    val s_axis = new streamdata_blackbox(AXIS_DATA_WIDTH, 1)
    val m_axis = Flipped(new streamdata_blackbox(AXIS_DATA_WIDTH, 1, NUM))
  })
}