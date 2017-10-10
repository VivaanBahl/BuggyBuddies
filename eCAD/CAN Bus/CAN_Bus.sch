<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="8.3.1">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="16" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="50" name="dxf" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="14" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="53" name="tGND_GNDA" color="7" fill="9" visible="no" active="no"/>
<layer number="54" name="bGND_GNDA" color="1" fill="9" visible="no" active="no"/>
<layer number="56" name="wert" color="7" fill="1" visible="no" active="no"/>
<layer number="57" name="tCAD" color="7" fill="1" visible="no" active="no"/>
<layer number="59" name="tCarbon" color="7" fill="1" visible="no" active="no"/>
<layer number="60" name="bCarbon" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="7" fill="1" visible="yes" active="yes"/>
<layer number="100" name="Muster" color="7" fill="1" visible="no" active="no"/>
<layer number="101" name="Patch_Top" color="12" fill="4" visible="yes" active="yes"/>
<layer number="102" name="Vscore" color="7" fill="1" visible="yes" active="yes"/>
<layer number="103" name="tMap" color="7" fill="1" visible="yes" active="yes"/>
<layer number="104" name="Name" color="16" fill="1" visible="yes" active="yes"/>
<layer number="105" name="tPlate" color="7" fill="1" visible="yes" active="yes"/>
<layer number="106" name="bPlate" color="7" fill="1" visible="yes" active="yes"/>
<layer number="107" name="Crop" color="7" fill="1" visible="yes" active="yes"/>
<layer number="108" name="tplace-old" color="10" fill="1" visible="yes" active="yes"/>
<layer number="109" name="ref-old" color="11" fill="1" visible="yes" active="yes"/>
<layer number="110" name="fp0" color="7" fill="1" visible="yes" active="yes"/>
<layer number="111" name="LPC17xx" color="7" fill="1" visible="yes" active="yes"/>
<layer number="112" name="tSilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="113" name="IDFDebug" color="4" fill="1" visible="yes" active="yes"/>
<layer number="114" name="Badge_Outline" color="7" fill="1" visible="yes" active="yes"/>
<layer number="115" name="ReferenceISLANDS" color="7" fill="1" visible="yes" active="yes"/>
<layer number="116" name="Patch_BOT" color="9" fill="4" visible="yes" active="yes"/>
<layer number="118" name="Rect_Pads" color="7" fill="1" visible="yes" active="yes"/>
<layer number="121" name="_tsilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="122" name="_bsilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="123" name="tTestmark" color="7" fill="1" visible="yes" active="yes"/>
<layer number="124" name="bTestmark" color="7" fill="1" visible="yes" active="yes"/>
<layer number="125" name="_tNames" color="7" fill="1" visible="yes" active="yes"/>
<layer number="126" name="_bNames" color="7" fill="1" visible="yes" active="yes"/>
<layer number="127" name="_tValues" color="7" fill="1" visible="yes" active="yes"/>
<layer number="128" name="_bValues" color="7" fill="1" visible="yes" active="yes"/>
<layer number="129" name="Mask" color="7" fill="1" visible="yes" active="yes"/>
<layer number="131" name="tAdjust" color="7" fill="1" visible="yes" active="yes"/>
<layer number="132" name="bAdjust" color="7" fill="1" visible="yes" active="yes"/>
<layer number="144" name="Drill_legend" color="7" fill="1" visible="yes" active="yes"/>
<layer number="150" name="Notes" color="7" fill="1" visible="yes" active="yes"/>
<layer number="151" name="HeatSink" color="7" fill="1" visible="yes" active="yes"/>
<layer number="152" name="_bDocu" color="7" fill="1" visible="yes" active="yes"/>
<layer number="153" name="FabDoc1" color="7" fill="1" visible="yes" active="yes"/>
<layer number="154" name="FabDoc2" color="7" fill="1" visible="yes" active="yes"/>
<layer number="155" name="FabDoc3" color="7" fill="1" visible="yes" active="yes"/>
<layer number="199" name="Contour" color="7" fill="1" visible="yes" active="yes"/>
<layer number="200" name="200bmp" color="1" fill="10" visible="yes" active="yes"/>
<layer number="201" name="201bmp" color="2" fill="10" visible="yes" active="yes"/>
<layer number="202" name="202bmp" color="3" fill="10" visible="yes" active="yes"/>
<layer number="203" name="203bmp" color="4" fill="10" visible="yes" active="yes"/>
<layer number="204" name="204bmp" color="5" fill="10" visible="yes" active="yes"/>
<layer number="205" name="205bmp" color="6" fill="10" visible="yes" active="yes"/>
<layer number="206" name="206bmp" color="7" fill="10" visible="yes" active="yes"/>
<layer number="207" name="207bmp" color="8" fill="10" visible="yes" active="yes"/>
<layer number="208" name="208bmp" color="9" fill="10" visible="yes" active="yes"/>
<layer number="209" name="209bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="210" name="210bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="211" name="211bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="212" name="212bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="213" name="213bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="214" name="214bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="215" name="215bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="216" name="216bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="217" name="217bmp" color="18" fill="1" visible="no" active="no"/>
<layer number="218" name="218bmp" color="19" fill="1" visible="no" active="no"/>
<layer number="219" name="219bmp" color="20" fill="1" visible="no" active="no"/>
<layer number="220" name="220bmp" color="21" fill="1" visible="no" active="no"/>
<layer number="221" name="221bmp" color="22" fill="1" visible="no" active="no"/>
<layer number="222" name="222bmp" color="23" fill="1" visible="no" active="no"/>
<layer number="223" name="223bmp" color="24" fill="1" visible="no" active="no"/>
<layer number="224" name="224bmp" color="25" fill="1" visible="no" active="no"/>
<layer number="225" name="225bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="226" name="226bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="227" name="227bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="228" name="228bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="229" name="229bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="230" name="230bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="231" name="231bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="232" name="Eagle3D_PG2" color="7" fill="1" visible="yes" active="yes"/>
<layer number="233" name="Eagle3D_PG3" color="7" fill="1" visible="yes" active="yes"/>
<layer number="248" name="Housing" color="7" fill="1" visible="yes" active="yes"/>
<layer number="249" name="Edge" color="7" fill="1" visible="yes" active="yes"/>
<layer number="250" name="Descript" color="3" fill="1" visible="no" active="no"/>
<layer number="251" name="SMDround" color="12" fill="11" visible="no" active="no"/>
<layer number="254" name="cooling" color="7" fill="1" visible="yes" active="yes"/>
<layer number="255" name="routoute" color="7" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="SparkFun-Connectors" urn="urn:adsk.eagle:library:513">
<description>&lt;h3&gt;SparkFun Connectors&lt;/h3&gt;
This library contains electrically-functional connectors. 
&lt;br&gt;
&lt;br&gt;
We've spent an enormous amount of time creating and checking these footprints and parts, but it is &lt;b&gt; the end user's responsibility&lt;/b&gt; to ensure correctness and suitablity for a given componet or application. 
&lt;br&gt;
&lt;br&gt;If you enjoy using this library, please buy one of our products at &lt;a href=" www.sparkfun.com"&gt;SparkFun.com&lt;/a&gt;.
&lt;br&gt;
&lt;br&gt;
&lt;b&gt;Licensing:&lt;/b&gt; Creative Commons ShareAlike 4.0 International - https://creativecommons.org/licenses/by-sa/4.0/ 
&lt;br&gt;
&lt;br&gt;
You are welcome to use this library for commercial purposes. For attribution, we ask that when you begin to sell your device using our footprint, you email us with a link to the product being sold. We want bragging rights that we helped (in a very small part) to create your 8th world wonder. We would like the opportunity to feature your device on our homepage.</description>
<packages>
<package name="DB9" urn="urn:adsk.eagle:footprint:37626/1" library_version="1">
<description>&lt;h3&gt;9 Pin Serial Connector - Female PCB Mount Right Angle&lt;/h3&gt;
0.318" style. 
&lt;p&gt;Specifications:
&lt;ul&gt;&lt;li&gt;Pin count:11&lt;/li&gt;
&lt;/ul&gt;&lt;/p&gt;
&lt;p&gt;&lt;a href=”https://www.sparkfun.com/datasheets/Prototyping/Connectors/DB9-PCB.pdf”&gt;Datasheet referenced for footprint&lt;/a&gt;&lt;/p&gt;
&lt;p&gt;Example device(s):
&lt;ul&gt;&lt;li&gt;DB9&lt;/li&gt;
&lt;/ul&gt;&lt;/p&gt;</description>
<wire x1="-8.128" y1="15.299" x2="-7.62" y2="15.807" width="0.2032" layer="51" curve="-90"/>
<wire x1="7.62" y1="15.807" x2="8.128" y2="15.299" width="0.2032" layer="51" curve="-90"/>
<wire x1="-15.4" y1="-2.8" x2="-15.4" y2="2.54" width="0.2032" layer="21"/>
<wire x1="-15.4" y1="2.5426" x2="-15.4" y2="9.5" width="0.2032" layer="51"/>
<wire x1="15.4" y1="-2.8" x2="15.4" y2="2.54" width="0.2032" layer="21"/>
<wire x1="15.4" y1="2.54" x2="15.4" y2="9.5" width="0.2032" layer="51"/>
<wire x1="-8.128" y1="10.219" x2="-8.128" y2="15.299" width="0.2032" layer="51"/>
<wire x1="-8.763" y1="9.584" x2="-8.128" y2="10.219" width="0.2032" layer="51" curve="90"/>
<wire x1="-15.4" y1="9.5" x2="15.4" y2="9.5" width="0.2032" layer="51"/>
<wire x1="8.128" y1="10.219" x2="8.128" y2="15.299" width="0.2032" layer="51"/>
<wire x1="8.128" y1="10.219" x2="8.763" y2="9.584" width="0.2032" layer="51" curve="90"/>
<wire x1="-7.62" y1="15.807" x2="7.62" y2="15.807" width="0.2032" layer="51"/>
<wire x1="-15.4" y1="-2.8" x2="15.4" y2="-2.8" width="0.2032" layer="21"/>
<pad name="1" x="-5.54" y="-1.42" drill="1.016" diameter="1.8796" shape="square"/>
<pad name="2" x="-2.77" y="-1.42" drill="1.016" diameter="1.8796"/>
<pad name="3" x="0" y="-1.42" drill="1.016" diameter="1.8796"/>
<pad name="4" x="2.77" y="-1.42" drill="1.016" diameter="1.8796"/>
<pad name="5" x="5.54" y="-1.42" drill="1.016" diameter="1.8796"/>
<pad name="6" x="-4.15" y="1.42" drill="1.016" diameter="1.8796"/>
<pad name="7" x="-1.38" y="1.42" drill="1.016" diameter="1.8796"/>
<pad name="8" x="1.38" y="1.42" drill="1.016" diameter="1.8796"/>
<pad name="9" x="4.15" y="1.42" drill="1.016" diameter="1.8796"/>
<pad name="G1" x="-12.5" y="0" drill="3.302" diameter="5.08"/>
<pad name="G2" x="12.5" y="0" drill="3.302" diameter="5.08"/>
<text x="-1.905" y="5.588" size="0.6096" layer="25" font="vector" ratio="20">&gt;NAME</text>
<text x="-1.905" y="4.572" size="0.6096" layer="27" font="vector" ratio="20">&gt;VALUE</text>
</package>
<package name="DB9_MALE" urn="urn:adsk.eagle:footprint:37627/1" library_version="1">
<description>&lt;h3&gt;9 Pin Serial Connector - Male PCB Mount Right Angle&lt;/h3&gt;
0.318" style. 
&lt;p&gt;Specifications:
&lt;ul&gt;&lt;li&gt;Pin count:11&lt;/li&gt;
&lt;/ul&gt;&lt;/p&gt;
&lt;p&gt;&lt;a href=”http://www.4uconnector.com/online/object/4udrawing/15944.pdf”&gt;Datasheet referenced for footprint&lt;/a&gt;&lt;/p&gt;
&lt;p&gt;Example device(s):
&lt;ul&gt;&lt;li&gt;DB9&lt;/li&gt;
&lt;/ul&gt;&lt;/p&gt;</description>
<wire x1="-8.128" y1="15.299" x2="-7.62" y2="15.807" width="0.2032" layer="51" curve="-90"/>
<wire x1="7.62" y1="15.807" x2="8.128" y2="15.299" width="0.2032" layer="51" curve="-90"/>
<wire x1="-15.4" y1="-2.8" x2="-15.4" y2="2.54" width="0.2032" layer="21"/>
<wire x1="-15.4" y1="2.5426" x2="-15.4" y2="9.5" width="0.2032" layer="51"/>
<wire x1="15.4" y1="-2.8" x2="15.4" y2="2.54" width="0.2032" layer="21"/>
<wire x1="15.4" y1="2.54" x2="15.4" y2="9.5" width="0.2032" layer="51"/>
<wire x1="-8.128" y1="10.219" x2="-8.128" y2="15.299" width="0.2032" layer="51"/>
<wire x1="-8.763" y1="9.584" x2="-8.128" y2="10.219" width="0.2032" layer="51" curve="90"/>
<wire x1="-15.4" y1="9.5" x2="15.4" y2="9.5" width="0.2032" layer="51"/>
<wire x1="8.128" y1="10.219" x2="8.128" y2="15.299" width="0.2032" layer="51"/>
<wire x1="8.128" y1="10.219" x2="8.763" y2="9.584" width="0.2032" layer="51" curve="90"/>
<wire x1="-7.62" y1="15.807" x2="7.62" y2="15.807" width="0.2032" layer="51"/>
<wire x1="-15.4" y1="-2.8" x2="15.4" y2="-2.8" width="0.2032" layer="21"/>
<pad name="5" x="-5.54" y="-1.42" drill="1.016" diameter="1.8796" shape="square"/>
<pad name="4" x="-2.77" y="-1.42" drill="1.016" diameter="1.8796"/>
<pad name="3" x="0" y="-1.42" drill="1.016" diameter="1.8796"/>
<pad name="2" x="2.77" y="-1.42" drill="1.016" diameter="1.8796"/>
<pad name="1" x="5.54" y="-1.42" drill="1.016" diameter="1.8796"/>
<pad name="9" x="-4.15" y="1.42" drill="1.016" diameter="1.8796"/>
<pad name="8" x="-1.38" y="1.42" drill="1.016" diameter="1.8796"/>
<pad name="7" x="1.38" y="1.42" drill="1.016" diameter="1.8796"/>
<pad name="6" x="4.15" y="1.42" drill="1.016" diameter="1.8796"/>
<pad name="G1" x="-12.5" y="0" drill="3.302" diameter="5.08"/>
<pad name="G2" x="12.5" y="0" drill="3.302" diameter="5.08"/>
<text x="-1.905" y="5.588" size="0.6096" layer="25" font="vector" ratio="20">&gt;NAME</text>
<text x="-1.905" y="4.572" size="0.6096" layer="27" font="vector" ratio="20">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="DB9" urn="urn:adsk.eagle:package:38025/1" type="box" library_version="1">
<description>9 Pin Serial Connector - Female PCB Mount Right Angle
0.318" style. 
Specifications:
Pin count:11

Datasheet referenced for footprint
Example device(s):
DB9
</description>
</package3d>
<package3d name="DB9_MALE" urn="urn:adsk.eagle:package:38021/1" type="box" library_version="1">
<description>9 Pin Serial Connector - Male PCB Mount Right Angle
0.318" style. 
Specifications:
Pin count:11

Datasheet referenced for footprint
Example device(s):
DB9
</description>
</package3d>
</packages3d>
<symbols>
<symbol name="F09G" urn="urn:adsk.eagle:symbol:37625/1" library_version="1">
<description>&lt;h3&gt;DB9 Connector&lt;/h3&gt;
Standard DB9 (D-Sub 9) serial connector. Can be used for CAN bus as well as RS485 and RS232.</description>
<wire x1="-1.651" y1="3.429" x2="-1.651" y2="1.651" width="0.254" layer="94" curve="180" cap="flat"/>
<wire x1="1.524" y1="1.651" x2="1.524" y2="3.429" width="0.254" layer="94" curve="180" cap="flat"/>
<wire x1="-1.651" y1="5.969" x2="-1.651" y2="4.191" width="0.254" layer="94" curve="180" cap="flat"/>
<wire x1="1.524" y1="-3.429" x2="1.524" y2="-1.651" width="0.254" layer="94" curve="180" cap="flat"/>
<wire x1="-1.651" y1="0.889" x2="-1.651" y2="-0.889" width="0.254" layer="94" curve="180" cap="flat"/>
<wire x1="1.524" y1="-0.889" x2="1.524" y2="0.889" width="0.254" layer="94" curve="180" cap="flat"/>
<wire x1="-1.651" y1="-1.651" x2="-1.651" y2="-3.429" width="0.254" layer="94" curve="180" cap="flat"/>
<wire x1="1.524" y1="-5.969" x2="1.524" y2="-4.191" width="0.254" layer="94" curve="180" cap="flat"/>
<wire x1="-1.651" y1="-4.191" x2="-1.651" y2="-5.969" width="0.254" layer="94" curve="180" cap="flat"/>
<wire x1="-4.064" y1="6.9312" x2="-2.5226" y2="8.172" width="0.4064" layer="94" curve="-102.324066" cap="flat"/>
<wire x1="-2.5226" y1="8.1718" x2="0" y2="7.62" width="0.4064" layer="94"/>
<wire x1="0" y1="7.62" x2="3.0654" y2="6.9494" width="0.4064" layer="94"/>
<wire x1="3.0654" y1="6.9495" x2="4.0642" y2="5.7088" width="0.4064" layer="94" curve="-77.655139" cap="flat"/>
<wire x1="4.064" y1="-5.7088" x2="4.064" y2="5.7088" width="0.4064" layer="94"/>
<wire x1="3.0654" y1="-6.9494" x2="4.064" y2="-5.7088" width="0.4064" layer="94" curve="77.657889"/>
<wire x1="-4.064" y1="-6.9312" x2="-4.064" y2="6.9312" width="0.4064" layer="94"/>
<wire x1="-2.5226" y1="-8.1718" x2="0" y2="-7.62" width="0.4064" layer="94"/>
<wire x1="0" y1="-7.62" x2="3.0654" y2="-6.9494" width="0.4064" layer="94"/>
<wire x1="-4.064" y1="-6.9312" x2="-2.5226" y2="-8.1719" width="0.4064" layer="94" curve="102.337599" cap="flat"/>
<text x="-3.81" y="-8.382" size="1.778" layer="96" font="vector" align="top-left">&gt;VALUE</text>
<text x="-3.81" y="8.636" size="1.778" layer="95" font="vector">&gt;NAME</text>
<pin name="1" x="-7.62" y="5.08" visible="pad" length="middle" direction="pas"/>
<pin name="6" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" rot="R180"/>
<pin name="2" x="-7.62" y="2.54" visible="pad" length="middle" direction="pas"/>
<pin name="7" x="7.62" y="0" visible="pad" length="middle" direction="pas" rot="R180"/>
<pin name="3" x="-7.62" y="0" visible="pad" length="middle" direction="pas"/>
<pin name="8" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" rot="R180"/>
<pin name="4" x="-7.62" y="-2.54" visible="pad" length="middle" direction="pas"/>
<pin name="9" x="7.62" y="-5.08" visible="pad" length="middle" direction="pas" rot="R180"/>
<pin name="5" x="-7.62" y="-5.08" visible="pad" length="middle" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="DB9" urn="urn:adsk.eagle:component:38317/1" prefix="J" uservalue="yes" library_version="1">
<description>&lt;h3&gt;DB9 Connector&lt;/h3&gt;
Standard DB9 (D-Sub 9) serial connector. Can be used for CAN bus as well as RS485 and RS232. Footprint production proven.

&lt;p&gt;&lt;/p&gt;
&lt;b&gt;Here is the female version connector we sell at SparkFun:&lt;/b&gt;
&lt;ul&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/429"&gt; 9 Pin Female Serial Connector - PCB Mount&lt;/a&gt; (PRT-00429)&lt;/li&gt;
&lt;/ul&gt;

&lt;p&gt;&lt;/p&gt;
&lt;b&gt;This connector has been used on the following SparkFun products:&lt;/b&gt;
&lt;ul&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/449"&gt;SparkFun RS232 Shifter - SMD&lt;/a&gt; (PRT-00449)&lt;/li&gt;
&lt;li&gt;&lt;a href="https://www.sparkfun.com/products/8780"&gt; SparkFun RS232 Shifter SMD (No DB9)&lt;/a&gt; (PRT-08780)&lt;/li&gt;
&lt;/ul&gt;</description>
<gates>
<gate name="G$1" symbol="F09G" x="0" y="0"/>
</gates>
<devices>
<device name="FEMALE" package="DB9">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:38025/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CONN-08194" constant="no"/>
</technology>
</technologies>
</device>
<device name="MALE" package="DB9_MALE">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:38021/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CONN-09904" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SparkFun-Resistors" urn="urn:adsk.eagle:library:532">
<description>&lt;h3&gt;SparkFun Resistors&lt;/h3&gt;
This library contains resistors. Reference designator:R. 
&lt;br&gt;
&lt;br&gt;
We've spent an enormous amount of time creating and checking these footprints and parts, but it is &lt;b&gt; the end user's responsibility&lt;/b&gt; to ensure correctness and suitablity for a given componet or application. 
&lt;br&gt;
&lt;br&gt;If you enjoy using this library, please buy one of our products at &lt;a href=" www.sparkfun.com"&gt;SparkFun.com&lt;/a&gt;.
&lt;br&gt;
&lt;br&gt;
&lt;b&gt;Licensing:&lt;/b&gt; Creative Commons ShareAlike 4.0 International - https://creativecommons.org/licenses/by-sa/4.0/ 
&lt;br&gt;
&lt;br&gt;
You are welcome to use this library for commercial purposes. For attribution, we ask that when you begin to sell your device using our footprint, you email us with a link to the product being sold. We want bragging rights that we helped (in a very small part) to create your 8th world wonder. We would like the opportunity to feature your device on our homepage.</description>
<packages>
<package name="AXIAL-0.3" urn="urn:adsk.eagle:footprint:39622/1" library_version="1">
<description>&lt;h3&gt;AXIAL-0.3&lt;/h3&gt;
&lt;p&gt;Commonly used for 1/4W through-hole resistors. 0.3" pitch between holes.&lt;/p&gt;</description>
<wire x1="-2.54" y1="0.762" x2="2.54" y2="0.762" width="0.2032" layer="21"/>
<wire x1="2.54" y1="0.762" x2="2.54" y2="0" width="0.2032" layer="21"/>
<wire x1="2.54" y1="0" x2="2.54" y2="-0.762" width="0.2032" layer="21"/>
<wire x1="2.54" y1="-0.762" x2="-2.54" y2="-0.762" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="-0.762" x2="-2.54" y2="0" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="0" x2="-2.54" y2="0.762" width="0.2032" layer="21"/>
<wire x1="2.54" y1="0" x2="2.794" y2="0" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="0" x2="-2.794" y2="0" width="0.2032" layer="21"/>
<pad name="P$1" x="-3.81" y="0" drill="0.9" diameter="1.8796"/>
<pad name="P$2" x="3.81" y="0" drill="0.9" diameter="1.8796"/>
<text x="0" y="1.016" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-1.016" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="AXIAL-0.3EZ" urn="urn:adsk.eagle:footprint:39624/1" library_version="1">
<description>This is the "EZ" version of the standard .3" spaced resistor package.&lt;br&gt;
It has a reduced top mask to make it harder to install upside-down.</description>
<wire x1="-2.54" y1="0.762" x2="2.54" y2="0.762" width="0.2032" layer="21"/>
<wire x1="2.54" y1="0.762" x2="2.54" y2="0" width="0.2032" layer="21"/>
<wire x1="2.54" y1="0" x2="2.54" y2="-0.762" width="0.2032" layer="21"/>
<wire x1="2.54" y1="-0.762" x2="-2.54" y2="-0.762" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="-0.762" x2="-2.54" y2="0" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="0" x2="-2.54" y2="0.762" width="0.2032" layer="21"/>
<wire x1="2.54" y1="0" x2="2.794" y2="0" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="0" x2="-2.794" y2="0" width="0.2032" layer="21"/>
<pad name="P$1" x="-3.81" y="0" drill="0.9" diameter="1.8796" stop="no"/>
<pad name="P$2" x="3.81" y="0" drill="0.9" diameter="1.8796" stop="no"/>
<text x="0" y="1.016" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-1.016" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
<circle x="-3.81" y="0" radius="0.508" width="0" layer="29"/>
<circle x="3.81" y="0" radius="0.523634375" width="0" layer="29"/>
<circle x="-3.81" y="0" radius="1.02390625" width="0" layer="30"/>
<circle x="3.81" y="0" radius="1.04726875" width="0" layer="30"/>
</package>
<package name="AXIAL-0.1" urn="urn:adsk.eagle:footprint:39620/1" library_version="1">
<description>&lt;h3&gt;AXIAL-0.1&lt;/h3&gt;
&lt;p&gt;Commonly used for 1/4W through-hole resistors. 0.1" pitch between holes.&lt;/p&gt;</description>
<wire x1="0" y1="-0.762" x2="0" y2="0" width="0.2032" layer="21"/>
<wire x1="0" y1="0" x2="0" y2="0.762" width="0.2032" layer="21"/>
<wire x1="0.254" y1="0" x2="0" y2="0" width="0.2032" layer="21"/>
<wire x1="0" y1="0" x2="-0.254" y2="0" width="0.2032" layer="21"/>
<pad name="P$1" x="-1.27" y="0" drill="0.9" diameter="1.8796"/>
<pad name="P$2" x="1.27" y="0" drill="0.9" diameter="1.8796"/>
<text x="0" y="1.143" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-1.143" size="0.6096" layer="21" font="vector" ratio="20" align="top-center">&gt;Value</text>
</package>
<package name="AXIAL-0.1-KIT" urn="urn:adsk.eagle:footprint:39621/1" library_version="1">
<description>&lt;h3&gt;AXIAL-0.1-KIT&lt;/h3&gt;
&lt;p&gt;Commonly used for 1/4W through-hole resistors. 0.1" pitch between holes.&lt;/p&gt;
&lt;p&gt;&lt;b&gt;Warning:&lt;/b&gt; This is the KIT version of the AXIAL-0.1 package. This package has a smaller diameter top stop mask, which doesn't cover the diameter of the pad. This means only the bottom side of the pads' copper will be exposed. You'll only be able to solder to the bottom side.&lt;/p&gt;</description>
<wire x1="0" y1="-0.762" x2="0" y2="0" width="0.2032" layer="21"/>
<wire x1="0" y1="0" x2="0" y2="0.762" width="0.2032" layer="21"/>
<wire x1="0.254" y1="0" x2="0" y2="0" width="0.2032" layer="21"/>
<wire x1="0" y1="0" x2="-0.254" y2="0" width="0.2032" layer="21"/>
<pad name="P$1" x="-1.27" y="0" drill="0.9" diameter="1.8796" stop="no"/>
<pad name="P$2" x="1.27" y="0" drill="0.9" diameter="1.8796" stop="no"/>
<text x="0" y="1.143" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;Name</text>
<text x="0" y="-1.143" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;Value</text>
<circle x="-1.27" y="0" radius="0.4572" width="0" layer="29"/>
<circle x="-1.27" y="0" radius="1.016" width="0" layer="30"/>
<circle x="1.27" y="0" radius="1.016" width="0" layer="30"/>
<circle x="-1.27" y="0" radius="0.4572" width="0" layer="29"/>
<circle x="1.27" y="0" radius="0.4572" width="0" layer="29"/>
</package>
<package name="0603" urn="urn:adsk.eagle:footprint:39615/1" library_version="1">
<description>&lt;p&gt;&lt;b&gt;Generic 1608 (0603) package&lt;/b&gt;&lt;/p&gt;
&lt;p&gt;0.2mm courtyard excess rounded to nearest 0.05mm.&lt;/p&gt;</description>
<wire x1="-1.6" y1="0.7" x2="1.6" y2="0.7" width="0.0508" layer="39"/>
<wire x1="1.6" y1="0.7" x2="1.6" y2="-0.7" width="0.0508" layer="39"/>
<wire x1="1.6" y1="-0.7" x2="-1.6" y2="-0.7" width="0.0508" layer="39"/>
<wire x1="-1.6" y1="-0.7" x2="-1.6" y2="0.7" width="0.0508" layer="39"/>
<wire x1="-0.356" y1="0.432" x2="0.356" y2="0.432" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.419" x2="0.356" y2="-0.419" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.1" dy="1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.1" dy="1" layer="1"/>
<text x="0" y="0.762" size="0.6096" layer="25" font="vector" ratio="20" align="bottom-center">&gt;NAME</text>
<text x="0" y="-0.762" size="0.6096" layer="27" font="vector" ratio="20" align="top-center">&gt;VALUE</text>
<rectangle x1="-0.8382" y1="-0.4699" x2="-0.3381" y2="0.4801" layer="51"/>
<rectangle x1="0.3302" y1="-0.4699" x2="0.8303" y2="0.4801" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
</packages>
<packages3d>
<package3d name="AXIAL-0.3" urn="urn:adsk.eagle:package:39658/1" type="box" library_version="1">
<description>AXIAL-0.3
Commonly used for 1/4W through-hole resistors. 0.3" pitch between holes.</description>
</package3d>
<package3d name="AXIAL-0.3EZ" urn="urn:adsk.eagle:package:39655/1" type="box" library_version="1">
<description>This is the "EZ" version of the standard .3" spaced resistor package.
It has a reduced top mask to make it harder to install upside-down.</description>
</package3d>
<package3d name="AXIAL-0.1" urn="urn:adsk.eagle:package:39656/1" type="box" library_version="1">
<description>AXIAL-0.1
Commonly used for 1/4W through-hole resistors. 0.1" pitch between holes.</description>
</package3d>
<package3d name="AXIAL-0.1-KIT" urn="urn:adsk.eagle:package:39653/1" type="box" library_version="1">
<description>AXIAL-0.1-KIT
Commonly used for 1/4W through-hole resistors. 0.1" pitch between holes.
Warning: This is the KIT version of the AXIAL-0.1 package. This package has a smaller diameter top stop mask, which doesn't cover the diameter of the pad. This means only the bottom side of the pads' copper will be exposed. You'll only be able to solder to the bottom side.</description>
</package3d>
<package3d name="0603" urn="urn:adsk.eagle:package:39650/1" type="box" library_version="1">
<description>Generic 1608 (0603) package
0.2mm courtyard excess rounded to nearest 0.05mm.</description>
</package3d>
</packages3d>
<symbols>
<symbol name="RESISTOR" urn="urn:adsk.eagle:symbol:39614/1" library_version="1">
<wire x1="-2.54" y1="0" x2="-2.159" y2="1.016" width="0.1524" layer="94"/>
<wire x1="-2.159" y1="1.016" x2="-1.524" y2="-1.016" width="0.1524" layer="94"/>
<wire x1="-1.524" y1="-1.016" x2="-0.889" y2="1.016" width="0.1524" layer="94"/>
<wire x1="-0.889" y1="1.016" x2="-0.254" y2="-1.016" width="0.1524" layer="94"/>
<wire x1="-0.254" y1="-1.016" x2="0.381" y2="1.016" width="0.1524" layer="94"/>
<wire x1="0.381" y1="1.016" x2="1.016" y2="-1.016" width="0.1524" layer="94"/>
<wire x1="1.016" y1="-1.016" x2="1.651" y2="1.016" width="0.1524" layer="94"/>
<wire x1="1.651" y1="1.016" x2="2.286" y2="-1.016" width="0.1524" layer="94"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="0" width="0.1524" layer="94"/>
<text x="0" y="1.524" size="1.778" layer="95" font="vector" align="bottom-center">&gt;NAME</text>
<text x="0" y="-1.524" size="1.778" layer="96" font="vector" align="top-center">&gt;VALUE</text>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="100OHM" urn="urn:adsk.eagle:component:39762/1" prefix="R" library_version="1">
<description>&lt;h3&gt;100Ω resistor&lt;/h3&gt;
&lt;p&gt;A resistor is a passive two-terminal electrical component that implements electrical resistance as a circuit element. Resistors act to reduce current flow, and, at the same time, act to lower voltage levels within circuits. - Wikipedia&lt;/p&gt;</description>
<gates>
<gate name="G$1" symbol="RESISTOR" x="0" y="0"/>
</gates>
<devices>
<device name="-HORIZ-1/4W-1%" package="AXIAL-0.3">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:39658/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="RES-12181" constant="no"/>
<attribute name="VALUE" value="100" constant="no"/>
</technology>
</technologies>
</device>
<device name="-HORIZ_KIT-1/4W-1%" package="AXIAL-0.3EZ">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:39655/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="RES-12181" constant="no"/>
<attribute name="VALUE" value="100" constant="no"/>
</technology>
</technologies>
</device>
<device name="-VERT-1/4W-1%" package="AXIAL-0.1">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:39656/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="RES-12181"/>
<attribute name="VALUE" value="100"/>
</technology>
</technologies>
</device>
<device name="-VERT_KIT-1/4W-1%" package="AXIAL-0.1-KIT">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:39653/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="RES-12181" constant="no"/>
<attribute name="VALUE" value="100" constant="no"/>
</technology>
</technologies>
</device>
<device name="-0603-1/4W-5%" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:39650/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="RES-12438"/>
<attribute name="VALUE" value="100"/>
</technology>
</technologies>
</device>
<device name="-0603-1/10W-1%" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:39650/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="RES-07863"/>
<attribute name="VALUE" value="100"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="R1" library="SparkFun-Resistors" library_urn="urn:adsk.eagle:library:532" deviceset="100OHM" device="-HORIZ-1/4W-1%" package3d_urn="urn:adsk.eagle:package:39658/1" value="150"/>
<part name="R2" library="SparkFun-Resistors" library_urn="urn:adsk.eagle:library:532" deviceset="100OHM" device="-HORIZ-1/4W-1%" package3d_urn="urn:adsk.eagle:package:39658/1" value="150"/>
<part name="J1" library="SparkFun-Connectors" library_urn="urn:adsk.eagle:library:513" deviceset="DB9" device="FEMALE" package3d_urn="urn:adsk.eagle:package:38025/1"/>
<part name="J2" library="SparkFun-Connectors" library_urn="urn:adsk.eagle:library:513" deviceset="DB9" device="FEMALE" package3d_urn="urn:adsk.eagle:package:38025/1"/>
<part name="J3" library="SparkFun-Connectors" library_urn="urn:adsk.eagle:library:513" deviceset="DB9" device="FEMALE" package3d_urn="urn:adsk.eagle:package:38025/1"/>
<part name="J4" library="SparkFun-Connectors" library_urn="urn:adsk.eagle:library:513" deviceset="DB9" device="FEMALE" package3d_urn="urn:adsk.eagle:package:38025/1"/>
<part name="J5" library="SparkFun-Connectors" library_urn="urn:adsk.eagle:library:513" deviceset="DB9" device="FEMALE" package3d_urn="urn:adsk.eagle:package:38025/1"/>
<part name="J6" library="SparkFun-Connectors" library_urn="urn:adsk.eagle:library:513" deviceset="DB9" device="FEMALE" package3d_urn="urn:adsk.eagle:package:38025/1"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="R1" gate="G$1" x="-71.12" y="40.64" rot="R90"/>
<instance part="R2" gate="G$1" x="109.22" y="43.18" rot="R90"/>
<instance part="J1" gate="G$1" x="-27.94" y="38.1"/>
<instance part="J2" gate="G$1" x="15.24" y="35.56"/>
<instance part="J3" gate="G$1" x="60.96" y="35.56"/>
<instance part="J4" gate="G$1" x="-27.94" y="63.5"/>
<instance part="J5" gate="G$1" x="17.78" y="60.96"/>
<instance part="J6" gate="G$1" x="63.5" y="60.96"/>
</instances>
<busses>
</busses>
<nets>
<net name="CAN_L" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="1"/>
<wire x1="-71.12" y1="35.56" x2="-66.04" y2="35.56" width="0.1524" layer="91"/>
<label x="-66.04" y="35.56" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="R2" gate="G$1" pin="1"/>
<wire x1="109.22" y1="38.1" x2="101.6" y2="38.1" width="0.1524" layer="91"/>
<label x="96.52" y="38.1" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="4"/>
<wire x1="-35.56" y1="35.56" x2="-40.64" y2="35.56" width="0.1524" layer="91"/>
<label x="-48.26" y="35.56" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J2" gate="G$1" pin="4"/>
<wire x1="7.62" y1="33.02" x2="2.54" y2="33.02" width="0.1524" layer="91"/>
<label x="-5.08" y="33.02" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J3" gate="G$1" pin="4"/>
<wire x1="53.34" y1="33.02" x2="48.26" y2="33.02" width="0.1524" layer="91"/>
<label x="40.64" y="33.02" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J4" gate="G$1" pin="4"/>
<wire x1="-35.56" y1="60.96" x2="-40.64" y2="60.96" width="0.1524" layer="91"/>
<label x="-48.26" y="60.96" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J5" gate="G$1" pin="4"/>
<wire x1="10.16" y1="58.42" x2="5.08" y2="58.42" width="0.1524" layer="91"/>
<label x="-2.54" y="58.42" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J6" gate="G$1" pin="4"/>
<wire x1="55.88" y1="58.42" x2="50.8" y2="58.42" width="0.1524" layer="91"/>
<label x="43.18" y="58.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="CAN_H" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="2"/>
<wire x1="-71.12" y1="45.72" x2="-66.04" y2="45.72" width="0.1524" layer="91"/>
<label x="-66.04" y="45.72" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="R2" gate="G$1" pin="2"/>
<wire x1="109.22" y1="48.26" x2="101.6" y2="48.26" width="0.1524" layer="91"/>
<label x="96.52" y="48.26" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="8"/>
<wire x1="-20.32" y1="35.56" x2="-15.24" y2="35.56" width="0.1524" layer="91"/>
<label x="-15.24" y="35.56" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J2" gate="G$1" pin="8"/>
<wire x1="22.86" y1="33.02" x2="27.94" y2="33.02" width="0.1524" layer="91"/>
<label x="27.94" y="33.02" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J3" gate="G$1" pin="8"/>
<wire x1="68.58" y1="33.02" x2="73.66" y2="33.02" width="0.1524" layer="91"/>
<label x="73.66" y="33.02" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J4" gate="G$1" pin="8"/>
<wire x1="-20.32" y1="60.96" x2="-15.24" y2="60.96" width="0.1524" layer="91"/>
<label x="-15.24" y="60.96" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J5" gate="G$1" pin="8"/>
<wire x1="25.4" y1="58.42" x2="30.48" y2="58.42" width="0.1524" layer="91"/>
<label x="30.48" y="58.42" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J6" gate="G$1" pin="8"/>
<wire x1="71.12" y1="58.42" x2="76.2" y2="58.42" width="0.1524" layer="91"/>
<label x="76.2" y="58.42" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="5"/>
<wire x1="-35.56" y1="33.02" x2="-40.64" y2="33.02" width="0.1524" layer="91"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="3"/>
<wire x1="-35.56" y1="38.1" x2="-40.64" y2="38.1" width="0.1524" layer="91"/>
<label x="-48.26" y="38.1" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J2" gate="G$1" pin="3"/>
<wire x1="7.62" y1="35.56" x2="2.54" y2="35.56" width="0.1524" layer="91"/>
<label x="-5.08" y="35.56" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J3" gate="G$1" pin="3"/>
<wire x1="53.34" y1="35.56" x2="48.26" y2="35.56" width="0.1524" layer="91"/>
<label x="40.64" y="35.56" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J4" gate="G$1" pin="3"/>
<wire x1="-35.56" y1="63.5" x2="-40.64" y2="63.5" width="0.1524" layer="91"/>
<label x="-48.26" y="63.5" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J5" gate="G$1" pin="3"/>
<wire x1="10.16" y1="60.96" x2="5.08" y2="60.96" width="0.1524" layer="91"/>
<label x="-2.54" y="60.96" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J6" gate="G$1" pin="3"/>
<wire x1="55.88" y1="60.96" x2="50.8" y2="60.96" width="0.1524" layer="91"/>
<label x="43.18" y="60.96" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="2"/>
<wire x1="-35.56" y1="40.64" x2="-40.64" y2="40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="5V" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="1"/>
<wire x1="-35.56" y1="43.18" x2="-40.64" y2="43.18" width="0.1524" layer="91"/>
<label x="-48.26" y="43.18" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J2" gate="G$1" pin="1"/>
<wire x1="7.62" y1="40.64" x2="2.54" y2="40.64" width="0.1524" layer="91"/>
<label x="-5.08" y="40.64" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J3" gate="G$1" pin="1"/>
<wire x1="53.34" y1="40.64" x2="48.26" y2="40.64" width="0.1524" layer="91"/>
<label x="40.64" y="40.64" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J4" gate="G$1" pin="1"/>
<wire x1="-35.56" y1="68.58" x2="-40.64" y2="68.58" width="0.1524" layer="91"/>
<label x="-48.26" y="68.58" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J5" gate="G$1" pin="1"/>
<wire x1="10.16" y1="66.04" x2="5.08" y2="66.04" width="0.1524" layer="91"/>
<label x="-2.54" y="66.04" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J6" gate="G$1" pin="1"/>
<wire x1="55.88" y1="66.04" x2="50.8" y2="66.04" width="0.1524" layer="91"/>
<label x="43.18" y="66.04" size="1.778" layer="95"/>
</segment>
</net>
<net name="3.3V" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="6"/>
<wire x1="-20.32" y1="40.64" x2="-15.24" y2="40.64" width="0.1524" layer="91"/>
<label x="-15.24" y="40.64" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J2" gate="G$1" pin="6"/>
<wire x1="22.86" y1="38.1" x2="27.94" y2="38.1" width="0.1524" layer="91"/>
<label x="27.94" y="38.1" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J3" gate="G$1" pin="6"/>
<wire x1="68.58" y1="38.1" x2="73.66" y2="38.1" width="0.1524" layer="91"/>
<label x="73.66" y="38.1" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J4" gate="G$1" pin="6"/>
<wire x1="-20.32" y1="66.04" x2="-15.24" y2="66.04" width="0.1524" layer="91"/>
<label x="-15.24" y="66.04" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J5" gate="G$1" pin="6"/>
<wire x1="25.4" y1="63.5" x2="30.48" y2="63.5" width="0.1524" layer="91"/>
<label x="30.48" y="63.5" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J6" gate="G$1" pin="6"/>
<wire x1="71.12" y1="63.5" x2="76.2" y2="63.5" width="0.1524" layer="91"/>
<label x="76.2" y="63.5" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="7"/>
<wire x1="-20.32" y1="38.1" x2="-15.24" y2="38.1" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="9"/>
<wire x1="-20.32" y1="33.02" x2="-15.24" y2="33.02" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="J2" gate="G$1" pin="5"/>
<wire x1="7.62" y1="30.48" x2="2.54" y2="30.48" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="J2" gate="G$1" pin="2"/>
<wire x1="7.62" y1="38.1" x2="2.54" y2="38.1" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="J2" gate="G$1" pin="7"/>
<wire x1="22.86" y1="35.56" x2="27.94" y2="35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="J2" gate="G$1" pin="9"/>
<wire x1="22.86" y1="30.48" x2="27.94" y2="30.48" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="J3" gate="G$1" pin="5"/>
<wire x1="53.34" y1="30.48" x2="48.26" y2="30.48" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="J3" gate="G$1" pin="2"/>
<wire x1="53.34" y1="38.1" x2="48.26" y2="38.1" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="J3" gate="G$1" pin="7"/>
<wire x1="68.58" y1="35.56" x2="73.66" y2="35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="J3" gate="G$1" pin="9"/>
<wire x1="68.58" y1="30.48" x2="73.66" y2="30.48" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="J4" gate="G$1" pin="5"/>
<wire x1="-35.56" y1="58.42" x2="-40.64" y2="58.42" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="J4" gate="G$1" pin="2"/>
<wire x1="-35.56" y1="66.04" x2="-40.64" y2="66.04" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="J4" gate="G$1" pin="7"/>
<wire x1="-20.32" y1="63.5" x2="-15.24" y2="63.5" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="J4" gate="G$1" pin="9"/>
<wire x1="-20.32" y1="58.42" x2="-15.24" y2="58.42" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="J5" gate="G$1" pin="5"/>
<wire x1="10.16" y1="55.88" x2="5.08" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<pinref part="J5" gate="G$1" pin="2"/>
<wire x1="10.16" y1="63.5" x2="5.08" y2="63.5" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="J5" gate="G$1" pin="7"/>
<wire x1="25.4" y1="60.96" x2="30.48" y2="60.96" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="J5" gate="G$1" pin="9"/>
<wire x1="25.4" y1="55.88" x2="30.48" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="J6" gate="G$1" pin="5"/>
<wire x1="55.88" y1="55.88" x2="50.8" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$22" class="0">
<segment>
<pinref part="J6" gate="G$1" pin="2"/>
<wire x1="55.88" y1="63.5" x2="50.8" y2="63.5" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="J6" gate="G$1" pin="7"/>
<wire x1="71.12" y1="60.96" x2="76.2" y2="60.96" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="J6" gate="G$1" pin="9"/>
<wire x1="71.12" y1="55.88" x2="76.2" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
