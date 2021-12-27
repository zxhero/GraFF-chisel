package utils
import chisel3._
import chisel3.util._

class fifo(val size : Int, val width : Int) extends Module {
  val io = IO(new Bundle {
    val dataIn = Input(UInt(width.W))
    val dataOut = Output(UInt(width.W))
    val writeFlag = Input(Bool())
    val readFlag = Input(Bool())
    val full = Output(Bool())
    val empty = Output(Bool())
    val rptr = Output(UInt((log2Ceil(size)+1).W))
  })

  val count = RegInit(0.U((log2Ceil(size)+1).W))
  val mem = Mem(size, UInt(width.W))
  val wPointer = RegInit(0.U((log2Ceil(size) + 1).W))
  val rPointer = RegInit(0.U((log2Ceil(size)+ 1).W))
  val dataOut = RegInit(0.U(width.W))

  io.rptr := rPointer

  def indexAdd(index : UInt) : UInt = {
      Mux(index === (size - 1).U, 0.U, index + 1.U)
  }

  def test_FIN() : Bool = {
    io.dataOut((width - 1).U) === 1.U && io.empty === false.B
  }

  when(io.writeFlag === true.B && io.readFlag === true.B) {
    when(count === 0.U) {
      mem(wPointer) := io.dataIn
      wPointer := indexAdd(wPointer)
      count := count + 1.U
      dataOut := 0.U
    }
    .otherwise {
      dataOut := mem(rPointer)
      rPointer := indexAdd(rPointer)
      mem(wPointer) := io.dataIn
      wPointer := indexAdd(wPointer)
    }
  } .elsewhen (io.writeFlag === true.B && io.readFlag === false.B) {
    dataOut := 0.U
    when(count < size.U) {
      mem(wPointer) := io.dataIn
      wPointer := indexAdd(wPointer)
      count := count + 1.U
    }
  } .elsewhen (io.writeFlag === false.B && io.readFlag === true.B) {
    when(count > 0.U) {
      dataOut := mem(rPointer)
      rPointer := indexAdd(rPointer)
      count := count - 1.U
    } .otherwise {
      dataOut := 0.U
    }
  } .otherwise {
    dataOut := 0.U
  }

  io.dataOut := dataOut
  io.full := (size.U === count)
  io.empty := (count === 0.U)
}

class regFile (val size : Int, val width : Int) extends Module {
  val io = IO(new Bundle {
    val dataIn = Input(UInt(width.W))
    val dataOut = Output(UInt(width.W))
    val writeFlag = Input(Bool())
    val rptr = Input(UInt((log2Ceil(size)).W))
    val wptr = Input(UInt((log2Ceil(size)).W))
  })

  val regs = RegInit(VecInit(Seq.fill(size)(0.U(width.W))))

  io.dataOut := regs(io.rptr)

  when(io.writeFlag){
    regs(io.wptr) := io.dataIn
  }
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
  })

  override def desiredName = mname
}