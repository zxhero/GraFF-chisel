package nf_arm_doce

import chisel3._
import chisel3.util.Decoupled
import chisel3.util._

class streamdata(AXIS_DATA_WIDTH: Int = 8, ELEMENT_WIDTH: Int = 1) extends Bundle {
  val tdata = Input(UInt((8 * AXIS_DATA_WIDTH).W))
  val tkeep = Input(UInt((AXIS_DATA_WIDTH / ELEMENT_WIDTH).W))
  val tlast = Input(Bool())
  val tvalid = Input(Bool())
  val tready = Output(Bool())

  def get_ith_data(i: Int): UInt = {
    assert(i < (AXIS_DATA_WIDTH / ELEMENT_WIDTH))
    tdata((i + 1) * ELEMENT_WIDTH * 8 - 1, i * ELEMENT_WIDTH * 8)
  }
}

class axidata(AXI_ADDR_WIDTH : Int = 44, AXI_DATA_WIDTH: Int = 16, AXI_ID_WIDTH: Int = 18, AXI_SIZE_WIDTH: Int = 3, NUM : Int = 1) extends Bundle {
  val awaddr = Input(UInt((NUM * AXI_ADDR_WIDTH).W))
  val awid = Input(UInt((NUM * AXI_ID_WIDTH).W))
  val awlen = Input(UInt((NUM * 8).W))
  val awsize = Input(UInt((NUM * AXI_SIZE_WIDTH).W))
  val awburst = Input(UInt((NUM * 2).W))
  val awlock = Input(UInt((NUM * 1).W))
  val awvalid = Input(UInt((NUM * 1).W))
  val awready = Output(UInt((NUM * 1).W))

  val araddr = Input(UInt((NUM * AXI_ADDR_WIDTH).W))
  val arid = Input(UInt((NUM * AXI_ID_WIDTH).W))
  val arlen = Input(UInt((NUM * 8).W))
  val arsize = Input(UInt((NUM * AXI_SIZE_WIDTH).W))
  val arburst = Input(UInt((NUM * 2).W))
  val arlock = Input(UInt((NUM * 1).W))
  val arvalid = Input(UInt((NUM * 1).W))
  val arready = Output(UInt((NUM * 1).W))

  val wdata = Input(UInt((NUM*8*AXI_DATA_WIDTH).W))
  val wstrb = Input(UInt((NUM*AXI_DATA_WIDTH).W))
  val wlast = Input(UInt((NUM * 1).W))
  val wvalid = Input(UInt((NUM * 1).W))
  val wready = Output(UInt((NUM * 1).W))

  val rdata = Output(UInt((NUM*8*AXI_DATA_WIDTH).W))
  val rid = Output(UInt((NUM * AXI_ID_WIDTH).W))
  val rlast = Output(UInt((NUM * 1).W))
  val rresp = Output(UInt((NUM * 2).W))
  val rvalid = Output(UInt((NUM * 1).W))
  val rready = Input(UInt((NUM * 1).W))

  val bresp = Output(UInt((NUM * 2).W))
  val bid = Output(UInt((NUM * AXI_ID_WIDTH).W))
  val bvalid = Output(UInt((NUM * 1).W))
  val bready = Input(UInt((NUM * 1).W))
}

class axilitedata(AXI_ADDR_WIDTH : Int = 44) extends Bundle {
  val awaddr = Input(UInt(AXI_ADDR_WIDTH.W))
  val awvalid = Input(Bool())
  val awready = Output(Bool())

  val araddr = Input(UInt(AXI_ADDR_WIDTH.W))
  val arvalid = Input(Bool())
  val arready = Output(Bool())

  val wdata = Input(UInt((32).W))
  val wstrb = Input(UInt(4.W))
  val wvalid = Input(Bool())
  val wready = Output(Bool())

  val rdata = Output(UInt((32).W))
  val rresp = Output(UInt(2.W))
  val rvalid = Output(Bool())
  val rready = Input(Bool())

  val bresp = Output(UInt(2.W))
  val bvalid = Output(Bool())
  val bready = Input(Bool())
}

class doceIO (AXI_ADDR_WIDTH : Int = 44, AXI_DATA_WIDTH: Int = 16, AXI_ID_WIDTH: Int = 18, AXI_SIZE_WIDTH: Int = 3) extends Bundle{
  val doce_axis_rxd = (new streamdata())
  val doce_axis_txd = Flipped((new streamdata()))
  val reset = Input(Bool())
  val clk = Input(Bool())
  val doce_axi_slave = new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)
  val doce_axi_master = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 4, AXI_SIZE_WIDTH))
  val doce_axi_lite_slave = new axilitedata(AXI_ADDR_WIDTH)
  val m_axi_doce_mac = Flipped(new axilitedata(AXI_ADDR_WIDTH))
  val doce_mac_addr = Input(UInt(48.W))
  val doce_ip_addr = Input(UInt(32.W))

  override def cloneType = new doceIO(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH).asInstanceOf[this.type]
}

class doce_top(AXI_ADDR_WIDTH : Int = 44, AXI_DATA_WIDTH: Int = 16, AXI_ID_WIDTH: Int = 18, AXI_SIZE_WIDTH: Int = 3) extends BlackBox{
	val io = IO(new doceIO(32, 16, 2, 3))

}

class mpsocIO () extends Bundle{
  val doce_axis_rxd = Flipped(new streamdata())
  val doce_axis_txd = ((new streamdata()))
  val gt_txp_out = Output(UInt(4.W))
  val gt_txn_out = Output(UInt(4.W))
  val gt_rxp_in = Input(UInt(4.W))
  val gt_rxn_in = Input(UInt(4.W))
  val gt_ref_clk_clk_p = Input(Bool())
  val gt_ref_clk_clk_n = Input(Bool())
  val doce_axi_lite_slave = Flipped(new axilitedata(40))
  val doce_axi_master = (new axidata(32, 16, 6, 3))
  val doce_axi_slave = Flipped(new axidata(40, 16, 0, 3))
  val doce_mac_addr = Output(UInt(48.W))
  val doce_ip_addr = Output(UInt(32.W))
  val gt_txusrclk = Output(Bool())
  val m_axi_doce_mac = (new axilitedata(32))
  val peripheral_reset = Output(Bool())
  val stat_rx_status_0 = Output(Bool())

  override def cloneType = new mpsocIO().asInstanceOf[this.type]
}

class mpsoc_wrapper() extends BlackBox{
  val io = IO(new mpsocIO())
}

class zynq_deoi_overlay_x86_xgbe() extends RawModule{
  val gt = IO(new Bundle{
    val txp_out = Output(UInt(4.W))
    val txn_out = Output(UInt(4.W))
    val rxp_in = Input(UInt(4.W))
    val rxn_in = Input(UInt(4.W))
    val ref_clk_clk_p = Input(Bool())
    val ref_clk_clk_n = Input(Bool())}
  )

  val mpsoc = Module(new mpsoc_wrapper())
  val doce = Module(new doce_top(32,16,2,3))

  mpsoc.io.doce_axis_rxd <> doce.io.doce_axis_rxd
  doce.io.doce_axis_txd <> mpsoc.io.doce_axis_txd
  mpsoc.io.m_axi_doce_mac <> doce.io.m_axi_doce_mac
  mpsoc.io.doce_axi_lite_slave <> doce.io.doce_axi_lite_slave
  mpsoc.io.doce_axi_master <> doce.io.doce_axi_master
  mpsoc.io.doce_axi_slave <> doce.io.doce_axi_slave
  doce.io.doce_ip_addr := mpsoc.io.doce_ip_addr
  doce.io.doce_mac_addr := mpsoc.io.doce_mac_addr
  doce.io.clk := mpsoc.io.gt_txusrclk
  doce.io.reset := mpsoc.io.peripheral_reset

  gt.txn_out := mpsoc.io.gt_txn_out
  gt.txp_out := mpsoc.io.gt_txp_out
  mpsoc.io.gt_rxp_in := gt.rxp_in
  mpsoc.io.gt_rxn_in := gt.rxn_in
  mpsoc.io.gt_ref_clk_clk_p := gt.ref_clk_clk_p
  mpsoc.io.gt_ref_clk_clk_n := gt.ref_clk_clk_n
}