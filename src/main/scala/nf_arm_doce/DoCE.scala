package doce

import chisel3._
import chisel3.util.Decoupled
import chisel3.util._
import nf_arm_doce.{axisdata, axisdata_u}

class pipeline[T <: Data](gen: T, val mname : String) extends Module{
  val io = IO(new Bundle() {
    val dout = Decoupled(gen)
    val din = Flipped(Decoupled(gen))
  })

  val data = RegInit(0.U.asTypeOf((gen)))
  val valid = RegInit(false.B)
  val ready = RegInit(false.B)
  val data2 = RegInit(0.U.asTypeOf((gen)))
  val valid2 = RegInit(false.B)
  //consumer takes 1 or valid is free
  when(!valid | io.dout.ready){
    //data2 produce 1
    when(valid2){
      data := data2
      valid := valid2
    }.elsewhen(io.din.ready){
      //din produce 1
      data := io.din.bits
      valid := io.din.valid
    }.otherwise{
      //no producer
      data := 0.U.asTypeOf((gen))
      valid := false.B
    }
  }
  ready := io.dout.ready

  //din produce 1 without consumer
  when(valid && !io.dout.ready && !valid2 ){
    data2 := io.din.bits
    valid2 := io.din.valid
  }.elsewhen(io.dout.ready){
    //consumer takes 1
    data2 := 0.U.asTypeOf((gen))
    valid2 := false.B
  }

  io.din.ready := ready | !valid | !valid2
  io.dout.valid := valid
  io.dout.bits := data

  override def desiredName = mname
}

//modules in transaction layer
class aw_width_converter(AW_Channel_Width : Int = 84, W_Channel_Width : Int = 144, Dout_Width : Int = 128) extends Module{
  val out_data = new Bundle() {
    val dout = UInt(Dout_Width.W)
    val keep = UInt((Dout_Width / 8).W)
    val last = Bool()
    val connection_id = UInt(4.W)
    val byte_num = UInt(13.W)
  }
  val io = IO(new Bundle() {
    val aw_channel = Flipped(Decoupled( new Bundle() {
      val aw = UInt(AW_Channel_Width.W)
      val connection_id = UInt(4.W)
    }))
    val w_channel = Flipped(Decoupled( new Bundle() {
      val w = UInt(W_Channel_Width.W)
      val last = Bool()
    }))
    val dout_channel = Decoupled(out_data)
  })
  assert(Dout_Width > AW_Channel_Width && (Dout_Width + Dout_Width / 8) == W_Channel_Width)

  val bit_sent = RegInit(0.U(32.W))
  val bit_recv = RegInit(0.U(32.W))
  val out = Module(new pipeline(out_data, "dout_channel_reg_slice"))
  val aw_len = io.aw_channel.bits.aw(77, 70)
  val mid = RegInit(0.U.asTypeOf(new Bundle() {
    val w = UInt(W_Channel_Width.W)
    val last = Bool()
  }))

  when(io.w_channel.ready && io.w_channel.valid){
    mid.w := io.w_channel.bits.w
    mid.last := io.w_channel.bits.last
  }

  when(out.io.din.valid && out.io.din.ready){
    when(out.io.din.bits.last){
      bit_sent := 0.U
    }.otherwise{
      bit_sent := bit_sent + Dout_Width.U
    }
  }

  when(io.w_channel.ready && io.w_channel.valid){
    when(io.aw_channel.ready && io.aw_channel.valid){
      bit_recv := bit_recv + W_Channel_Width.U + 128.U
    }.otherwise{
      bit_recv := bit_recv + W_Channel_Width.U
    }
  }.elsewhen(out.io.din.valid && out.io.din.ready && out.io.din.bits.last){
    bit_recv := 0.U
  }

  io.dout_channel <> out.io.dout
  out.io.din.valid := false.B
  out.io.din.bits.dout := 0.U
  out.io.din.bits.last := false.B
  out.io.din.bits.keep := 0.U
  out.io.din.bits.connection_id := 0.U
  out.io.din.bits.byte_num := 0.U
  io.aw_channel.ready := false.B
  io.w_channel.ready := false.B
  val left = bit_recv - bit_sent
  //left i bits from the last beat
  for(i <- 0 to W_Channel_Width by (Dout_Width / 8)){
    when(left === i.U){
      if(i == 0){
        when(bit_sent === 0.U && io.aw_channel.valid && io.w_channel.valid){
          if(Dout_Width == 128){
            out.io.din.bits.dout := io.aw_channel.bits.aw.asTypeOf(UInt(128.W))
          }else {
            out.io.din.bits.dout := Cat(io.w_channel.bits.w(Dout_Width - 128 - 1, 0),
              io.aw_channel.bits.aw.asTypeOf(UInt(128.W)))
          }
          out.io.din.valid := true.B
          out.io.din.bits.last := false.B
          out.io.din.bits.keep := VecInit(Seq.fill(Dout_Width / 8)(true.B)).asUInt()
          out.io.din.bits.connection_id := io.aw_channel.bits.connection_id
          out.io.din.bits.byte_num := 16.U + (W_Channel_Width / 8).U
          io.aw_channel.ready := out.io.din.ready
          io.w_channel.ready := out.io.din.ready
        }.elsewhen(bit_sent > 0.U && io.w_channel.valid){
          out.io.din.valid := true.B
          out.io.din.bits.dout := io.w_channel.bits.w(Dout_Width - 1, 0)
          out.io.din.bits.last := false.B
          out.io.din.bits.keep := VecInit(Seq.fill(Dout_Width / 8)(true.B)).asUInt()
          io.w_channel.ready := out.io.din.ready
        }
      }else if(i < Dout_Width){
        when(mid.last){
          out.io.din.valid := true.B
          out.io.din.bits.dout := mid.w(W_Channel_Width - 1, W_Channel_Width - i)
          out.io.din.bits.last := true.B
          out.io.din.bits.keep := VecInit(Seq.fill((i / 8))(true.B)).asUInt()
        }.elsewhen(io.w_channel.valid){
          out.io.din.valid := true.B
          out.io.din.bits.dout := Cat(io.w_channel.bits.w(Dout_Width - i - 1, 0), mid.w(W_Channel_Width - 1, W_Channel_Width - i))
          out.io.din.bits.last := false.B
          out.io.din.bits.keep := VecInit(Seq.fill(Dout_Width / 8)(true.B)).asUInt()
          io.w_channel.ready := out.io.din.ready
        }
      }else if(i >= Dout_Width){
        out.io.din.valid := true.B
        out.io.din.bits.dout := mid.w(W_Channel_Width - i + Dout_Width - 1, W_Channel_Width - i)
        out.io.din.bits.last := false.B
        out.io.din.bits.keep := VecInit(Seq.fill(Dout_Width / 8)(true.B)).asUInt()
      }
    }
  }
}

class aw_decode(AW_Channel_Width : Int = 80, W_Channel_Width : Int = 144, Din_Width : Int = 128) extends Module{
  val io = IO(new Bundle() {
    val aw_channel = (Decoupled( new Bundle() {
      val aw = UInt(AW_Channel_Width.W)
    }))
    val w_channel = (Decoupled( new Bundle() {
      val w = UInt(W_Channel_Width.W)
      val last = Bool()
    }))
    val din_channel = Flipped(Decoupled(new Bundle() {
      val din = UInt(Din_Width.W)
      val last = Bool()
    }))
    val phy_base_0 = Input(UInt(49.W))
    val phy_base_1 = Input(UInt(49.W))
  })
  assert(Din_Width > AW_Channel_Width && (Din_Width + Din_Width / 8) == W_Channel_Width)

  val bit_recv = RegInit(0.U(32.W))
  val bit_sent = RegInit(0.U(32.W))
  val aw_channel_out = Module(new pipeline(new Bundle() {
    val aw = UInt(AW_Channel_Width.W)
  }, "aw_channel_out_reg_slice"))
  val w_channel_out = Module(new pipeline(new Bundle() {
    val w = UInt(W_Channel_Width.W)
    val last = Bool()
  }, "w_channel_out_reg_slice"))
  val mid = RegInit(0.U.asTypeOf(new Bundle() {
    val w = UInt(Din_Width.W)
    val last = Bool()
  }))

  when(io.din_channel.ready && io.din_channel.valid){
    when(io.din_channel.bits.last){
      bit_recv := 0.U
    }.otherwise{
      bit_recv := bit_recv + Din_Width.U
    }
  }

  when(w_channel_out.io.din.valid && w_channel_out.io.din.ready){
    when(w_channel_out.io.din.bits.last){
      bit_sent := 0.U
    }.otherwise{
      bit_sent := bit_sent + W_Channel_Width.U
    }
  }.elsewhen(aw_channel_out.io.din.valid && aw_channel_out.io.din.ready){
    bit_sent := bit_sent + 128.U
  }

  when(io.din_channel.ready && io.din_channel.valid){
    mid.w := io.din_channel.bits.din
    mid.last := io.din_channel.bits.last
  }

  io.aw_channel <> aw_channel_out.io.dout
  io.w_channel <> w_channel_out.io.dout

  io.din_channel.ready := false.B
  w_channel_out.io.din.valid := false.B
  w_channel_out.io.din.bits.w := 0.U
  w_channel_out.io.din.bits.last := false.B
  aw_channel_out.io.din.valid := false.B
  val left = bit_recv - bit_sent
  //left i bits from the last beat
  for(i <- 0 to Din_Width by (Din_Width / 8)){
    when(left === i.U){
      if(i == 0){
        when(bit_sent === 0.U && io.din_channel.valid){
          aw_channel_out.io.din.valid := true.B
          io.din_channel.ready := aw_channel_out.io.din.ready
        }.elsewhen(io.din_channel.valid){
          io.din_channel.ready := true.B
        }
      }else{
        when(io.din_channel.valid){
          io.din_channel.ready := w_channel_out.io.din.ready
          w_channel_out.io.din.valid := true.B
          w_channel_out.io.din.bits.w := Cat(io.din_channel.bits.din(W_Channel_Width - i - 1, 0), mid.w(Din_Width - 1, Din_Width - i))
          w_channel_out.io.din.bits.last := io.din_channel.bits.last
        }
      }
    }
  }

  aw_channel_out.io.din.bits.aw := Cat(io.din_channel.bits.din(83, 8), io.din_channel.bits.din(3, 0))
  when(io.din_channel.bits.din(7, 4) === io.phy_base_0(47, 44) && io.phy_base_0(48)){
    aw_channel_out.io.din.bits.aw := Cat(io.din_channel.bits.din(83, 8), io.din_channel.bits.din(3, 0)) |
      Cat(io.phy_base_0(43, 0), 0.U(4.W)).asTypeOf(UInt(AW_Channel_Width.W))
  }.elsewhen(io.din_channel.bits.din(7, 4) === io.phy_base_1(47, 44) && io.phy_base_1(48)){
    aw_channel_out.io.din.bits.aw := Cat(io.din_channel.bits.din(83, 8), io.din_channel.bits.din(3, 0)) |
      Cat(io.phy_base_1(43, 0), 0.U(4.W)).asTypeOf(UInt(AW_Channel_Width.W))
  }
}

//modules in transport layer
class reg_slice[T <: Data](gen: T,val mname : String) extends Module{
  val io = IO(new Bundle() {
    val dout = Decoupled(gen)
    val din = Flipped(Decoupled(gen))
  })

  val data = RegInit(0.U.asTypeOf(gen))
  val valid = RegInit(false.B)
  when(io.dout.ready || !valid){
    data := io.din.bits
    valid := io.din.valid
  }

  io.din.ready := io.dout.ready || !valid
  io.dout.valid := valid
  io.dout.bits := data

  override def desiredName = mname
}

class tx_packeting(AXIS_DATA_WIDTH : Int = 16) extends Module{
  val io = IO(new Bundle() {
    val axis_rd_from_fsm = Flipped(Decoupled( new axisdata(AXIS_DATA_WIDTH)))
    val axi_str_to_router = (Decoupled( new axisdata(AXIS_DATA_WIDTH)))
  })
  val CMD_Width = 16
  assert(AXIS_DATA_WIDTH >= CMD_Width)

  if(AXIS_DATA_WIDTH == CMD_Width){
    io.axi_str_to_router <> io.axis_rd_from_fsm
  }else{
    val router_out = Module(new reg_slice(new axisdata(AXIS_DATA_WIDTH), "router_out_reg_slice"))
    router_out.io.din.bits.tkeep := io.axis_rd_from_fsm.bits.tkeep
    router_out.io.din.bits.tdata := io.axis_rd_from_fsm.bits.tdata
    router_out.io.din.bits.tlast := io.axis_rd_from_fsm.bits.tlast
    router_out.io.din.valid := io.axis_rd_from_fsm.valid
    io.axis_rd_from_fsm.ready := router_out.io.din.ready
    when(io.axis_rd_from_fsm.bits.tlast &&
      io.axis_rd_from_fsm.bits.tkeep(AXIS_DATA_WIDTH - 1, AXIS_DATA_WIDTH - CMD_Width) === 0.U){
      when(io.axi_str_to_router.valid & io.axi_str_to_router.ready){
        router_out.io.din.bits.tkeep := 0.U
        router_out.io.din.bits.tdata := 0.U
        router_out.io.din.bits.tlast := false.B
        router_out.io.din.valid := false.B
        io.axis_rd_from_fsm.ready := true.B
      }.otherwise{
        router_out.io.din.bits.tkeep := 0.U
        router_out.io.din.bits.tdata := 0.U
        router_out.io.din.bits.tlast := false.B
        router_out.io.din.valid := false.B
        io.axis_rd_from_fsm.ready := false.B
      }
    }

    io.axi_str_to_router.valid := io.axis_rd_from_fsm.valid && router_out.io.dout.valid
    io.axi_str_to_router.bits.tlast := io.axis_rd_from_fsm.bits.tlast &&
      io.axis_rd_from_fsm.bits.tkeep(AXIS_DATA_WIDTH - 1, AXIS_DATA_WIDTH - CMD_Width) === 0.U
    io.axi_str_to_router.bits.tdata := Cat(io.axis_rd_from_fsm.bits.tdata((AXIS_DATA_WIDTH - CMD_Width) * 8 - 1, 0),
      router_out.io.dout.bits.tdata(AXIS_DATA_WIDTH * 8 - 1, (AXIS_DATA_WIDTH - CMD_Width) * 8))
    io.axi_str_to_router.bits.tkeep := Cat(io.axis_rd_from_fsm.bits.tkeep(AXIS_DATA_WIDTH - CMD_Width - 1, 0),
      router_out.io.dout.bits.tkeep(AXIS_DATA_WIDTH - 1, AXIS_DATA_WIDTH - CMD_Width))
    when(router_out.io.dout.bits.tlast){
      io.axi_str_to_router.bits.tdata := router_out.io.dout.bits.tdata(AXIS_DATA_WIDTH * 8 - 1, (AXIS_DATA_WIDTH - CMD_Width) * 8)
      io.axi_str_to_router.bits.tkeep := router_out.io.dout.bits.tkeep(AXIS_DATA_WIDTH - 1, AXIS_DATA_WIDTH - CMD_Width)
      io.axi_str_to_router.valid := router_out.io.dout.valid
      io.axi_str_to_router.bits.tlast := true.B
    }

    router_out.io.dout.ready := io.axi_str_to_router.valid & io.axi_str_to_router.ready
  }
}

class rx_depacketing(AXIS_DATA_WIDTH : Int = 16) extends Module{
  val io = IO(new Bundle() {
    val axis_str_to_trans = (Decoupled( new axisdata_u(AXIS_DATA_WIDTH, 4)))
    val axi_str_from_router = Flipped(Decoupled( new axisdata_u(AXIS_DATA_WIDTH, 4)))
  })
  val CMD_Width = 16
  assert(AXIS_DATA_WIDTH >= CMD_Width)

  if(AXIS_DATA_WIDTH == CMD_Width){
    io.axis_str_to_trans <> io.axi_str_from_router
  }else{
    val trans_out = Module(new reg_slice(new axisdata_u(AXIS_DATA_WIDTH, 4), "trans_out_reg_slice"))
    trans_out.io.din <> io.axi_str_from_router

    io.axis_str_to_trans.valid := Mux(trans_out.io.dout.bits.tlast, trans_out.io.dout.valid,
      io.axi_str_from_router.valid && trans_out.io.dout.valid)
    io.axis_str_to_trans.bits.tlast := trans_out.io.dout.bits.tlast
    io.axis_str_to_trans.bits.tuser := trans_out.io.dout.bits.tuser
    io.axis_str_to_trans.bits.tdata := Cat(io.axi_str_from_router.bits.tdata((CMD_Width) * 8 - 1, 0),
      trans_out.io.dout.bits.tdata(AXIS_DATA_WIDTH * 8 - 1, (CMD_Width) * 8))
    io.axis_str_to_trans.bits.tkeep := Cat(io.axi_str_from_router.bits.tkeep(CMD_Width - 1, 0),
      trans_out.io.dout.bits.tkeep(AXIS_DATA_WIDTH - 1, CMD_Width))
    trans_out.io.dout.ready := io.axis_str_to_trans.valid & io.axis_str_to_trans.ready
  }
}