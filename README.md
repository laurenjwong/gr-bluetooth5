# gr-bluetooth5

Implementation of a receiver for Bluetooth 5.0 advertising packets in GNU Radio. The custom built module receives Bluetooth 5.0 packets on the three available advertising channels, parses the I/Q samples for advertising frames, and further parses the found frames for the frame elements. This implementation was tested on using file captures of both Bluetooth 4 and 5 devices taken from a LightBlue Bean and iPhone 6s taken using an Ettus USRP B200 software defined radio.

Three custom blocks make up the gr-bluetooth5 module:
 - The *parse* block  is  a  GNU  Radio  Out  of  Tree  Module,  and  contains  the  majority  of  the  Bluetooth
advertising packet parsing implementation written in Python. As input, the parse block takes in the demodulated symbols from the available GFSK demod block and the channel number to tune to. As output, the parse block prints the contents of the found and parsed packets to the the terminal.
 - The *btle5_singleChannel* is a hierarchical  block  containing  a GFSK demod block and one parse blocks. It takes in a channel and source of raw samples, and outputs the contents of the found and parsed packets to the terminal.
  - The *btle5_allChannels* block is also hierarchical  block  containing  a GFSK demod block, but contains three parse blocks, one for each of the advertising channels. It takes as input only a source of raw samples, and outputs the contents of the found and parsed packets to the terminal.
  
