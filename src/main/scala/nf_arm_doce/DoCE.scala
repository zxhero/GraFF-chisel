package doce

import chisel3._
import chisel3.util.Decoupled
import chisel3.util._
import nf_arm_doce.{axisdata, axisdata_u}
import utils.axis_reg_slice

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

  val bit_sent = RegInit(0.U(16.W))
  val out = Module(new axis_reg_slice(Dout_Width / 8 + 3, "dout_channel_reg_slice"))
  out.io.aclk := clock.asBool()
  out.io.aresetn := ~reset.asBool()
  val aw_in = Module(new axis_reg_slice(11, "aw_channel_reg_slice"))
  aw_in.io.aclk := clock.asBool()
  aw_in.io.aresetn := ~reset.asBool()
  aw_in.io.s_axis.tdata := Cat(io.aw_channel.bits.aw, io.aw_channel.bits.connection_id)
  aw_in.io.s_axis.tvalid := io.aw_channel.valid
  io.aw_channel.ready := aw_in.io.s_axis.tready
  val w_in = Module(new axis_reg_slice(W_Channel_Width / 8, "w_channel_reg_slice"))
  w_in.io.aclk := clock.asBool()
  w_in.io.aresetn := ~reset.asBool()
  w_in.io.s_axis.tdata := io.w_channel.bits.w
  w_in.io.s_axis.tlast := io.w_channel.bits.last
  w_in.io.s_axis.tvalid := io.w_channel.valid
  io.w_channel.ready := w_in.io.s_axis.tready
  val aw_len = aw_in.io.m_axis.tdata(77 + 4, 70 + 4)
  val mid = RegInit(0.U.asTypeOf(new Bundle() {
    val w = UInt(W_Channel_Width.W)
    val last = Bool()
  }))
  val mid_count = RegInit(0.U(16.W))
  when((w_in.io.m_axis.tready & w_in.io.m_axis.tvalid).asBool()){
    when((aw_in.io.m_axis.tready & aw_in.io.m_axis.tvalid).asBool()){
      mid_count := (W_Channel_Width + 128 - Dout_Width).U
    }.otherwise{
      mid_count := mid_count + (W_Channel_Width - Dout_Width).U
    }
  }.elsewhen((out.io.s_axis.tvalid & out.io.s_axis.tready).asBool()){
    when(out.io.s_axis.tlast.asBool()){
      mid_count := 0.U
    }.otherwise{
      mid_count := mid_count - Dout_Width.U
    }
  }

  when((w_in.io.m_axis.tready & w_in.io.m_axis.tvalid).asBool()){
    mid.w := w_in.io.m_axis.tdata
    mid.last := w_in.io.m_axis.tlast
  }

  when((out.io.s_axis.tvalid & out.io.s_axis.tready).asBool()){
    when(out.io.s_axis.tlast.asBool()){
      bit_sent := 0.U
    }.otherwise{
      bit_sent := bit_sent + Dout_Width.U
    }
  }

  io.dout_channel.valid := out.io.m_axis.tvalid
  io.dout_channel.bits.dout := out.io.m_axis.tdata(Dout_Width + 16, 17)
  io.dout_channel.bits.last := out.io.m_axis.tlast
  io.dout_channel.bits.keep := out.io.m_axis.tkeep
  io.dout_channel.bits.byte_num := out.io.m_axis.tdata(12, 0)
  io.dout_channel.bits.connection_id := out.io.m_axis.tdata(16, 13)
  out.io.m_axis.tready := io.dout_channel.ready

  val out_tmp = Wire(out_data)
  out.io.s_axis.tdata := Cat(out_tmp.dout, Cat(out_tmp.connection_id, out_tmp.byte_num))
  out.io.s_axis.tkeep := out_tmp.keep
  out.io.s_axis.tlast := out_tmp.last
  out.io.s_axis.tvalid := false.B
  out_tmp.dout := 0.U
  out_tmp.last := false.B
  out_tmp.keep := 0.U
  out_tmp.connection_id := 0.U
  out_tmp.byte_num := 0.U
  w_in.io.m_axis.tready := false.B
  aw_in.io.m_axis.tready := false.B
  //left i bits from the last beat
  for(i <- 0 to W_Channel_Width by (Dout_Width / 8)){
    when(mid_count === i.U){
      if(i == 0){
        when(bit_sent === 0.U && aw_in.io.m_axis.tvalid.asBool() && w_in.io.m_axis.tvalid.asBool()){
          if(Dout_Width == 128){
            out_tmp.dout := aw_in.io.m_axis.tdata(AW_Channel_Width + 3, 4).asTypeOf(UInt(128.W))
          }else {
            out_tmp.dout := Cat(w_in.io.m_axis.tdata(Dout_Width - 128 - 1, 0),
              aw_in.io.m_axis.tdata(AW_Channel_Width + 3, 4).asTypeOf(UInt(128.W)))
          }
          out.io.s_axis.tvalid := true.B
          out_tmp.last := false.B
          out_tmp.keep := VecInit(Seq.fill(Dout_Width / 8)(true.B)).asUInt()
          out_tmp.connection_id := aw_in.io.m_axis.tdata(3, 0)
          out_tmp.byte_num := 16.U + (W_Channel_Width / 8).U
          aw_in.io.m_axis.tready := out.io.s_axis.tready
          w_in.io.m_axis.tready := out.io.s_axis.tready
        }.elsewhen(bit_sent > 0.U && w_in.io.m_axis.tvalid.asBool()){
          out.io.s_axis.tvalid := true.B
          out_tmp.dout := w_in.io.m_axis.tdata(Dout_Width - 1, 0)
          out_tmp.last := false.B
          out_tmp.keep := VecInit(Seq.fill(Dout_Width / 8)(true.B)).asUInt()
          w_in.io.m_axis.tready := out.io.s_axis.tready
        }
      }else if(i < Dout_Width){
        when(mid.last){
          out.io.s_axis.tvalid := true.B
          out_tmp.dout := mid.w(W_Channel_Width - 1, W_Channel_Width - i)
          out_tmp.last := true.B
          out_tmp.keep := VecInit(Seq.fill((i / 8))(true.B)).asUInt()
        }.elsewhen(w_in.io.m_axis.tvalid.asBool()){
          out.io.s_axis.tvalid := true.B
          out_tmp.dout := Cat(w_in.io.m_axis.tdata(Dout_Width - i - 1, 0), mid.w(W_Channel_Width - 1, W_Channel_Width - i))
          out_tmp.last := false.B
          out_tmp.keep := VecInit(Seq.fill(Dout_Width / 8)(true.B)).asUInt()
          w_in.io.m_axis.tready := out.io.s_axis.tready
        }
      }else if(i >= Dout_Width){
        out.io.s_axis.tvalid := true.B
        out_tmp.dout := mid.w(W_Channel_Width - i + Dout_Width - 1, W_Channel_Width - i)
        out_tmp.last := false.B
        out_tmp.keep := VecInit(Seq.fill(Dout_Width / 8)(true.B)).asUInt()
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

  val bit_sent = RegInit(0.U(16.W))
  val in = Module(new axis_reg_slice(Din_Width / 8, "aw_decode_in_reg_slice"))
  in.io.aclk := clock.asBool()
  in.io.aresetn := ~reset.asBool()
  in.io.s_axis.tvalid := io.din_channel.valid
  in.io.s_axis.tdata := io.din_channel.bits.din
  in.io.s_axis.tlast := io.din_channel.bits.last
  io.din_channel.ready := in.io.s_axis.tready
  val aw_channel_out = Module(new axis_reg_slice(AW_Channel_Width / 8, "aw_channel_out_reg_slice"))
  aw_channel_out.io.aclk := clock.asBool()
  aw_channel_out.io.aresetn := ~reset.asBool()
  val w_channel_out = Module(new axis_reg_slice(W_Channel_Width / 8, "w_channel_out_reg_slice"))
  w_channel_out.io.aclk := clock.asBool()
  w_channel_out.io.aresetn := ~reset.asBool()
  val mid = RegInit(0.U.asTypeOf(new Bundle() {
    val w = UInt(Din_Width.W)
    val last = Bool()
  }))
  val mid_count = RegInit(0.U(16.W))
  when((in.io.m_axis.tvalid & in.io.m_axis.tready).asBool()){
    when(mid_count === 0.U){
      when(bit_sent === 0.U){
        mid_count := (Din_Width - 128).U
      }.otherwise{
        mid_count := Din_Width.U
      }
    }.otherwise{
      when(in.io.m_axis.tlast.asBool()){
        mid_count := 0.U
      }.otherwise{
        mid_count := mid_count - (W_Channel_Width - Din_Width).U
      }
    }
  }

  when((w_channel_out.io.s_axis.tvalid & w_channel_out.io.s_axis.tready).asBool()){
    when(w_channel_out.io.s_axis.tlast.asBool()){
      bit_sent := 0.U
    }.otherwise{
      bit_sent := bit_sent + W_Channel_Width.U
    }
  }.elsewhen((aw_channel_out.io.s_axis.tvalid & aw_channel_out.io.s_axis.tready).asBool()){
    bit_sent := bit_sent + 128.U
  }

  when((in.io.m_axis.tvalid & in.io.m_axis.tready).asBool()){
    mid.w := in.io.m_axis.tdata
    mid.last := in.io.m_axis.tlast
  }

  io.aw_channel.valid := aw_channel_out.io.m_axis.tvalid
  io.aw_channel.bits.aw := aw_channel_out.io.m_axis.tdata
  aw_channel_out.io.m_axis.tready := io.aw_channel.ready
  io.w_channel.valid := w_channel_out.io.m_axis.tvalid
  io.w_channel.bits.last := w_channel_out.io.m_axis.tlast
  io.w_channel.bits.w := w_channel_out.io.m_axis.tdata
  w_channel_out.io.m_axis.tready := io.w_channel.ready

  in.io.m_axis.tready := false.B
  w_channel_out.io.s_axis.tvalid := false.B
  w_channel_out.io.s_axis.tdata := 0.U
  w_channel_out.io.s_axis.tlast := false.B
  aw_channel_out.io.s_axis.tvalid := false.B
  //left i bits from the last beat
  for(i <- 0 to Din_Width by (Din_Width / 8)){
    when(mid_count === i.U){
      if(i == 0){
        when(bit_sent === 0.U && in.io.m_axis.tvalid.asBool()){
          aw_channel_out.io.s_axis.tvalid := true.B
          in.io.m_axis.tready := aw_channel_out.io.s_axis.tready
        }.elsewhen(in.io.m_axis.tvalid.asBool()){
          in.io.m_axis.tready := true.B
        }
      }else{
        when(in.io.m_axis.tvalid.asBool()){
          in.io.m_axis.tready := w_channel_out.io.s_axis.tready
          w_channel_out.io.s_axis.tvalid := true.B
          w_channel_out.io.s_axis.tdata := Cat(in.io.m_axis.tdata(W_Channel_Width - i - 1, 0), mid.w(Din_Width - 1, Din_Width - i))
          w_channel_out.io.s_axis.tlast := in.io.m_axis.tlast
        }
      }
    }
  }

  aw_channel_out.io.s_axis.tdata := Cat(in.io.m_axis.tdata(83, 8), in.io.m_axis.tdata(3, 0))
  when(in.io.m_axis.tdata(7, 4) === io.phy_base_0(47, 44) && io.phy_base_0(48)){
    aw_channel_out.io.s_axis.tdata := Cat(in.io.m_axis.tdata(83, 8), in.io.m_axis.tdata(3, 0)) |
      Cat(io.phy_base_0(43, 0), 0.U(4.W)).asTypeOf(UInt(AW_Channel_Width.W))
  }.elsewhen(in.io.m_axis.tdata(7, 4) === io.phy_base_1(47, 44) && io.phy_base_1(48)){
    aw_channel_out.io.s_axis.tdata := Cat(in.io.m_axis.tdata(83, 8), in.io.m_axis.tdata(3, 0)) |
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