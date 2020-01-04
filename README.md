# gr-bluetooth5

Implementation of a receiver for Bluetooth 5.0 advertising packets in GNU Radio. The custom built module receives Bluetooth 5.0 packets on the three available advertising channels, parses the I/Q samples for advertising frames, and further parses the found frames for the frame elements.
Three custom blocks make up the gr-bluetooth5 module:
 - The parse block  is  a  GNU  Radio  Out  of  Tree  Module,  and  contains  the  majority  of  the  Bluetooth
advertising packet parsing implementation written in Python. As input, the parse block takes in the demodulated symbols from the available GFSK demod block and the channel number to tune to. As output, the parse block prints the contents of the found and parsed packets to the the terminal.
 - The btle5_singleChannel and btle5_allChannels blocks are both hierarchical  blocks  containing  a GFSK demod block and one or more parse blocks.
