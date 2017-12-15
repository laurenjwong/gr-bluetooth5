#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2017 <+YOU OR YOUR COMPANY+>.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

import numpy as np
from gnuradio import gr
import scipy.signal
import time   
import copy

def btle_lfsr(data_in, whitener):
	state = whitener
	data_out = np.zeros((len(data_in),))
	for k in range(0, len(data_in)):
		feedback = state[6]
		data_out[k] = np.logical_xor(data_in[k], feedback)
		state[6] = state[5]
		state[5] = state[4]
		state[4] = np.logical_xor(state[3], feedback)
		state[3] = state[2]
		state[2] = state[1]
		state[1] = state[0]
		state[0] = feedback
	return data_out, state

def btle_crc(data_in, init):
	state = init
	for kk in range(0, len(data_in)):
		feedback = np.logical_xor(data_in[kk], state[23])
		state[23] = state[22]
		state[22] = state[21]
		state[21] = state[20]
		state[20] = state[19]
		state[19] = state[18]
		state[18] = state[17]
		state[17] = state[16]
		state[16] = state[15]
		state[15] = state[14]
		state[14] = state[13]
		state[13] = state[12]
		state[12] = state[11]
		state[11] = state[10]
		state[10] = np.logical_xor(state[9], feedback)
		state[9] = np.logical_xor(state[8], feedback)
		state[8] = state[7]
		state[7] = state[6]
		state[6] = np.logical_xor(state[5], feedback)
		state[5] = state[4]
		state[4] = np.logical_xor(state[3], feedback)
		state[3] = np.logical_xor(state[2], feedback)
		state[2] = state[1]
		state[1] = np.logical_xor(state[0], feedback)
		state[0] = feedback
	return state

def byte_flip(in_bytes):
	out_bytes = np.zeros((len(in_bytes),))
	kk = np.arange(0,len(in_bytes), 8)
	for k in kk:
		out_bytes[k:k+8] = np.fliplr([in_bytes[k:k+8]])[0]
	return out_bytes

def pdu_parse(frame):
	pdu_type = frame['header'][0:4]
	if(np.array_equal(pdu_type, [0,0,0,0])):
		frame['pdu_type'] = 'ADV IND'
		frame['adva'] = np.fliplr([frame['payload'][0:48]])[0]
		frame['adva_hex'] = binvect2hexstr(frame['adva'],0)
		frame['adv_data'] = byte_flip(frame['payload'][48:-24])
		frame['adv_data_hex'] = binvect2hexstr(frame['adv_data'],0)
	elif(np.array_equal(pdu_type, [1,0,0,0])):
		frame['pdu_type'] = 'ADV DIRECT IND'
		frame['adva'] = np.fliplr([frame['payload'][0:48]])[0]
		frame['adva_hex'] = binvect2hexstr(frame['adva'],0)
		frame['adv_data'] = byte_flip(frame['payload'][48:-24])
		frame['adv_data_hex'] = binvect2hexstr(frame['adv_data'],0)
	elif(np.array_equal(pdu_type, [0,1,0,0])):
		frame['pdu_type'] = 'ADV NONCONN IND'
		frame['adva'] = np.fliplr([frame['payload'][0:48]])[0]
		frame['adva_hex'] = binvect2hexstr(frame['adva'],0)
		frame['adv_data'] = byte_flip(frame['payload'][48:-24])
		frame['adv_data_hex'] = binvect2hexstr(frame['adv_data'],0)
	elif(np.array_equal(pdu_type, [1,1,0,0])):
		frame['pdu_type'] = 'SCAN REQ'
		frame['adva'] = np.fliplr([frame['payload'][0:48]])[0]
		frame['adva_hex'] = binvect2hexstr(frame['adva'],0)
		frame['adv_data'] = byte_flip(frame['payload'][48:-24])
		frame['adv_data_hex'] = binvect2hexstr(frame['adv_data'],0)
	elif(np.array_equal(pdu_type, [0,0,1,0])):
		frame['pdu_type'] = 'SCAN RSP'
		frame['adva'] = np.fliplr([frame['payload'][0:48]])[0]
		frame['adva_hex'] = binvect2hexstr(frame['adva'],0)
		frame['adv_data'] = byte_flip(frame['payload'][48:-24])
		frame['adv_data_hex'] = binvect2hexstr(frame['adv_data'],0)
	elif(np.array_equal(pdu_type, [1,0,1,0])):
		frame['pdu_type'] = 'CONNECT REQ'
		frame['adva'] = np.fliplr([frame['payload'][0:48]])[0]
		frame['adva_hex'] = binvect2hexstr(frame['adva'],0)
		frame['adv_data'] = byte_flip(frame['payload'][48:-24])
		frame['adv_data_hex'] = binvect2hexstr(frame['adv_data'],0)
	elif(np.array_equal(pdu_type, [0,1,1,0])):
		frame['pdu_type'] = 'ADVSCAN IND'
		frame['adva'] = np.fliplr([frame['payload'][0:48]])[0]
		frame['adva_hex'] = binvect2hexstr(frame['adva'],0)
		frame['adv_data'] = byte_flip(frame['payload'][48:-24])
		frame['adv_data_hex'] = binvect2hexstr(frame['adv_data'],0)
	elif(np.array_equal(pdu_type, [1,1,1,0])):
		frame['pdu_type'] = 'ADV_EXT_IND'
		header_len = num2str(frame['payload'][0:6])
		header_len = header_len.replace(' ','')
		header_len = int(header_len,2)
		frame['adva'] = np.fliplr([frame['payload'][16:16+(6*8)]])[0]
		frame['adva_hex'] = binvect2hexstr(frame['adva'],0)
		frame['adv_data'] = byte_flip(frame['payload'][8+header_len:-27])
		frame['adv_data_hex'] = binvect2hexstr(frame['adv_data'],0)	
	else:
		frame['pdu_type'] = 'PDU TYPE NOT FOUND'
	frame['crc'] = byte_flip(frame['payload'][-24:])
	frame['crc_hex'] = binvect2hexstr(frame['crc'],0)
	return frame

def num2str(bin_vect):
	bin_str = ''
	for i in bin_vect:
		bin_str = bin_str + ' ' + str(int(i))
	return bin_str

def binvect2hexstr(bin_vect, flip_toggle):
	if(np.array_equal(bin_vect, [])):
		return '0x'
	else:
		bin_str = ''
		kk = np.arange(0,len(bin_vect), 8)
		for k in kk:
			if(flip_toggle == 0):
				tmp = num2str(bin_vect[k:k+8])
			else:
				tmp = str(np.fliplr([bin_vect[k:k+8]])[0])
			tmp = tmp.replace(' ','')
			bin_str = bin_str + tmp
		hex_str = hex(int(bin_str, 2))[2:]
		hex_str = '0x' + hex_str 
		return hex_str  

def demap(data, P):
	if (P == 4):
		data_map = {'0011':0, '1100':1}
	else:
		data_map = {'0':0, '1':1}
	out_data = np.zeros((int(len(data)/P),))
	kk = np.arange(0,len(data), P)
	ii = 0
	for k in kk:
		out_data[ii] = data_map[str(data[k:k+P])]
		ii = ii + 1
	return out_data

def viterbi(data):
	## To Be Implemented
	return data

def decode(data, S):
	data = demap(data, S/2)
	decoded_data = viterbi(data)
	return decoded_data

class parse(gr.sync_block):
	"""
	Parses data stream of IQ samples for BTLE advertising frames.
	"""
	def __init__(self, channel):
		gr.sync_block.__init__(self,
			name="parse",
			in_sig=[np.int8],
			out_sig=[])
		## Advertising Channel Params (Defined in Core Specifications)
		self.preamble1 = [0,1,0,1,0,1,0,1]
		self.preamble2 = [0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1]
		self.preamble3 =[0,0,1,1,1,1,0,0,
							0,0,1,1,1,1,0,0,
							0,0,1,1,1,1,0,0,
							0,0,1,1,1,1,0,0,
							0,0,1,1,1,1,0,0,
							0,0,1,1,1,1,0,0,
							0,0,1,1,1,1,0,0,
							0,0,1,1,1,1,0,0,
							0,0,1,1,1,1,0,0,
							0,0,1,1,1,1,0,0,]
		self.access_address = [0,1,1,0,1,0,1,1,0,1,1,1,1,1,0,1,1,0,0,1,0,0,0,1,0,1,1,1,0,0,0,1]

		self.channel = channel

		if(self.channel == 37):
			self.lfsr_init = [1,1,0,0,1,0,1]
		elif(self.channel == 38):
			self.lfsr_init = [1,1,0,0,1,1,0]
		elif(self.channel == 39):
			self.lfsr_init = [1,1,0,0,1,1,1]
		else:
			raise ValueError('Invalid channel entered.')

		self.crc_init = np.fliplr([[0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1]])[0]

		## For display purposes
		self.display_frame = ['reserved_1', 'tx_add', 'rx_add', 'reserved_2', 'payload_len_dec', 'pdu_type', 'adva_hex', 'adv_data_hex', 'crc_hex', 'crc_rx_hex', 'crc_flag', 'total_len_dec']

	def forecast(self, ninput_items_required):
		ninput_items_required[0] = 4096

	def work(self, input_items, output_items):
		in0 = input_items[0]
		out = output_items
        
		data = input_items[0][0:4096]
		for ii in range(0, len(data)-56):
			## Legacy or 1M PHY: If Preamble and Access Address are found, continue parsing the frame.
			if(np.array_equal(data[ii:ii+8],self.preamble1) and np.array_equal(data[ii+8:ii+40],self.access_address)):
				frame = {}
				## Extract the Header and Length fields from the frame.
				frame['raw_data'] = data[ii:]
				frame['header'], frame['lfsr_state'] = btle_lfsr(frame['raw_data'][40:56], copy.deepcopy(self.lfsr_init))
				frame['header'][8:16] = byte_flip(frame['header'][8:16])

				frame['reserved_1'] = frame['header'][4:6]
				frame['tx_add'] = frame['header'][6]
				frame['rx_add'] = frame['header'][7]

				frame['reserved_2'] = frame['header'][8:10]
				payload_len = frame['header'][10:16]
				frame['payload_len_dec'] = num2str(payload_len)
				frame['payload_len_dec'] = frame['payload_len_dec'].replace(' ','')
				frame['payload_len_dec'] = int(frame['payload_len_dec'],2)

				## Extract the Payload fields from the frame.
				index = 56+(frame['payload_len_dec']+3)*8
				frame['payload'], discard = btle_lfsr(frame['raw_data'][56:index], frame['lfsr_state'])
				frame['raw_data'] = frame['raw_data'][0:index]
				
				frame['pdu'], discard = btle_lfsr(frame['raw_data'][40:index], copy.deepcopy(self.lfsr_init))
				frame = pdu_parse(frame)

				## Calculate the expected CRC and compare against the received CRC.
				crc_rx = np.fliplr([byte_flip(btle_crc(frame['pdu'][0:-24], copy.deepcopy(self.crc_init)))])[0]
				frame['crc_rx_hex'] = binvect2hexstr(crc_rx, 0)

				if(np.array_equal(frame['crc'], crc_rx)):
					frame['crc_flag'] = 1
				else:
					frame['crc_flag'] = 0

				frame = pdu_parse(frame)

				frame['total_len_dec'] = 8+32+8+8+(8*frame['payload_len_dec'])+24

				## Output frame, if CRC is valid. Report if invalid frame is found.
				if(frame['crc_flag'] != 0):
					print('1M PHY Frame Found on Channel ' + str(self.channel) + ':')
					for key in self.display_frame:
						print(key + ': ' + str(frame[key]))
					print 
				
			## 2M PHY: If Preamble and Access Address are found, continue parsing the frame.
			elif(np.array_equal(data[ii:ii+16],self.preamble2) and np.array_equal(data[ii+16:ii+48],self.access_address)):
				frame = {}
				## Extract the Header and Length fields from the frame.
				frame['raw_data'] = data[ii:]
				frame['header'], frame['lfsr_state'] = btle_lfsr(frame['raw_data'][48:64], copy.deepcopy(self.lfsr_init))
				frame['header'][16:24] = byte_flip(frame['header'][16:24])

				frame['reserved_1'] = frame['header'][4:6]
				frame['tx_add'] = frame['header'][6]
				frame['rx_add'] = frame['header'][7]

				frame['reserved_2'] = frame['header'][8:10]
				payload_len = frame['header'][10:16]
				frame['payload_len_dec'] = num2str(payload_len)
				frame['payload_len_dec'] = frame['payload_len_dec'].replace(' ','')
				frame['payload_len_dec'] = int(frame['payload_len_dec'],2)

				## Extract the Payload fields from the frame.
				index = 64+(frame['payload_len_dec']+3)*8
				frame['payload'], discard = btle_lfsr(frame['raw_data'][64:index], frame['lfsr_state'])
				frame['raw_data'] = frame['raw_data'][0:index]
				
				frame['pdu'], discard = btle_lfsr(frame['raw_data'][48:index], copy.deepcopy(self.lfsr_init))
				frame = pdu_parse(frame)

				## Calculate the expected CRC and compare against the received CRC.
				crc_rx = np.fliplr([byte_flip(btle_crc(frame['pdu'][0:-24], copy.deepcopy(self.crc_init)))])[0]
				frame['crc_rx_hex'] = binvect2hexstr(crc_rx, 0)

				if(np.array_equal(frame['crc'], crc_rx)):
					frame['crc_flag'] = 1
				else:
					frame['crc_flag'] = 0

				frame = pdu_parse(frame)

				frame['total_len_dec'] = 8+32+8+8+(8*frame['payload_len_dec'])+24

				## Output frame, if CRC is valid. Report if invalid frame is found.
				if(frame['crc_flag'] != 0):
					print('2M PHY Frame Found on Channel ' + str(self.channel) + ':')
					for key in self.display_frame:
						print(key + ': ' + str(frame[key]))
					print 

			## Coded PHY: If Preamble and Access Address are found, continue parsing the frame.			
			elif(np.array_equal(data[ii:ii+80],self.preamble3)):
				print('Coded PHY Frame Found on Channel' + str(self.channel) + '. Decoding needed.')

				## Backend implementation to extract packet data. Viterbi decoding implementation needed.
#				frame = {}
#				frame['raw_data'] = data[ii:]

#				## FEC Block 1
#				frame['coding_index'] = decode(frame['raw_data'][80+32:80+32+2], 8)
#				frame['term1'] = decode(frame['raw_data'][80+32+2:80+32+2+3], 8)

#				## FEC Block 2
#				pdu_header, frame['lfsr_state'] = btle_lfsr(frame['raw_data'][80+32+2+3:80+32+2+3+16],copy.deepcopy(self.lfsr_init))
#				frame['header'] = decode(pdu_header, frame['coding_index'])

#				## Header Info
#				frame['reserved'] = frame['header'][4]
#				frame['ch_sel'] = frame['header'][5]
#				frame['tx_add'] = frame['header'][6]
#				frame['rx_add'] = frame['header'][7]
#				payload_len = frame['header'][8:16]
#				frame['payload_len_dec'] = num2str(payload_len)
#				frame['payload_len_dec'] = frame['payload_len_dec'].replace(' ','')
#				frame['payload_len_dec'] = int(frame['payload_len_dec'],2)
#				
#				## Extract the Payload fields from the frame.
#				index = 80+32+2+3+16+(frame['payload_len_dec']+3)*8
#				frame['raw_data'] = frame['raw_data'][0:index]
#				payload, discard = btle_lfsr(frame['raw_data'][80+32+2+3+16:index], frame['lfsr_state'])
#				frame['payload'] = decode(payload, frame['coding_index'])
#				pdu, discard = btle_lfsr(frame['raw_data'][80+32+2+3:index], copy.deepcopy(self.lfsr_init))
#				frame['pdu'] = decode(pdu, frame['coding_index'])
#				frame = pdu_parse(frame)

#				## Overwrite CRC calculation, not the same as in 1M and 2M.
#				frame['crc'] = byte_flip(frame['payload'][-27:-3])
#				frame['crc_hex'] = binvect2hexstr(frame['crc'],0)

#				## Calculate the expected CRC and compare against the received CRC. Get term2.
#				temp = frame['pdu'][0:-24]
#				crc_rx_coded = np.fliplr([byte_flip(btle_crc(temp[0:24], copy.deepcopy(self.crc_init)))])[0]
#				crc_rx = decode(crc_rx_coded, frame['coding_index'])
#				frame['crc_rx_hex'] = binvect2hexstr(crc_rx, 0)
#				
#				frame['term2'] = decode(temp[24:], frame['coding_index'])

#				if(np.array_equal(frame['crc'], crc_rx)):
#					frame['crc_flag'] = 1
#				else:
#					frame['crc_flag'] = 0

#				frame = pdu_parse(frame)

#				frame['total_len_dec'] = ((80+32+2+3+16)-8)+(8*frame['payload_len_dec'])+24+3

#				## Output frame, if CRC is valid. Report if invalid frame is found.
#				if(frame['crc_flag'] != 0):
#					print('Coded PHY Frame Found on Channel' + str(self.channel) + ':')
#					for key in self.display_frame:
#						print(key + ': ' + str(frame[key]))
#					print 
#				else:
#					print('Invalid Frame!') 
#					print('payload_len_dec: ' + str(frame['payload_len_dec']))
#					print('crc_hex: '+ str(frame['crc_hex']))

		self.consume_each(4096)
		return len(output_items)

