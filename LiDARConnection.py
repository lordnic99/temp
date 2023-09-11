from socket import timeout
import serial
import time

#Thông tin cổng Serial
SERIAL_PORT = "COM3"
SERIAL_BAUDRATE = 230400

#Scan variables
scanSamplesSignalQuality= [0.0]
scanSamplesRange = [0.0]

#Giá trị mặc định khung truyền
FRAME_HEADER = 0xAA		
PROTOCOL_VERSION = 0x01		
FRAME_TYPE = 0x61			

#Đọc 16 frame để lấy được khung truyền hoàn chỉnh

SO_LAN_SCAN = 15

#Chuyển đổi các thông số
CHUYEN_TOC_DO_DONG_CO = 0.05 * 60 		#Chuyển từ rad /s sang RPM
CHUYEN_SANG_GOC = 0.01			
KHOANG_CACH = 0.25 * 0.001				#Chuyển tự mm sang m

#Cau trúc khung truyền
class Delta2AFrame:
	frameHeader = 0			#Frame Header: 1 byte
	frameLength = 0			#Frame Length: 2 bytes
	protocolVersion = 0		#Protocol Version: 1 byte
	frameType = 0			#Frame Type: 1 byte
	commandWord = 0			#Command Word: 1 byte
	parameterLength = 0		#Parameter Length: 2 bytes
	parameters = [0]		#Parameter
	check_code = 0			#Check code: 2 bytes

def LiDARFrameProcessing(frame: Delta2GFrame):
	match frame.commandWord:
		case 0xAE:
			#Thông báo lỗi
			rpm = frame.parameters[0] * CHUYEN_TOC_DO_DONG_CO
			print("RPM: %f" % rpm)
		case 0xAD:
			#bước 1: Tốc độ động cơ
			rpm = frame.parameters[0] * ROTATION_SPEED_SCALE
			print("RPM: %f" % rpm)

			#bước 2: offset
			offsetAngle = (frame.parameters[1] << 8) + frame.parameters[2]
			offsetAngle = offsetAngle * ANGLE_SCALE

			#bước 3: Góc bắt đầu khung truyền
			startAngle = (frame.parameters[3] << 8) + frame.parameters[4]
			startAngle = startAngle * ANGLE_SCALE

			#Tổng số điểm trong khung truyền
			sampleCnt = int((frame.parameterLength - 5) / 3)

			#Khung truyền nào trong 16 khung truyền
			frameIndex = int(startAngle / (360.0 / SCAN_STEPS))

			if frameIndex == 0:
				#Bắt đầu quét 
				scanSamplesRange.clear()
				scanSamplesSignalQuality.clear()

			#4th: LiDAR samples, each sample has: Signal Value/Quality (1 byte), Distance Value (2 bytes)
			for i in range(sampleCnt):
				signalQuality = frame.parameters[5 + (i * 3)]
				distance = (frame.parameters[5 + (i * 3) + 1] << 8) + frame.parameters[5 + (i * 3) + 2]
				scanSamplesSignalQuality.append(signalQuality)
				scanSamplesRange.append(distance * RANGE_SCALE)

			if frameIndex == (SCAN_STEPS - 1):
				#Scan complete
				print("SCAN: %d" % len(scanSamplesRange))

def main():
	try:
		lidarSerial = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0)
	except serial.serialutil.SerialException:
		print("Không thể kết nối với Lidar!")
		exit()

	trang_thai = 0
	check_code = 0
	lidarFrame = Delta2AFrame()
	while True:
		du_lieu_nhan_duoc = lidarSerial.read(100) #Đọc 100 byte

		for byte in du_lieu_nhan_duoc:
			match trang_thai:
				case 0:
					#Tìm byte Frame Header
					lidarFrame.frameHeader = byte
					if lidarFrame.frameHeader == FRAME_HEADER:
						#Tìm thấy FRAME_HEADER
						trang_thai = 1
					else:
						print("Không tìm thấy FRAME_HEADER!")
					#Reset lại check code, tiếp tục đọc khung truyền
					check_code = 0
				case 1:
					#FRAME LENGTH bit cao
					lidarFrame.frameLength = (byte << 8)
					trang_thai = 2
				case 2:
					#FRAME LENGTH bit thấp
					lidarFrame.frameLength += byte
					trang_thai = 3
				case 3:
					#PROTOCOL VERSION
					lidarFrame.protocolVersion = byte
					if lidarFrame.protocolVersion == PROTOCOL_VERSION:
						#Kiểm tra protocol version
						trang_thai = 4
					else:
						print("Sai Protocol Version")
						trang_thai = 0
				case 4:
					#FRAME TYPE
					lidarFrame.frameType = byte
					if lidarFrame.frameType == FRAME_TYPE:
						#Kiểm tra FRAME_TYPE
						trang_thai = 5
					else:
						print("Sai Frame Type")
						trang_thai = 0
				case 5:
					#COMMAND
					lidarFrame.commandWord = byte
					trang_thai = 6
				case 6:
					#PARAMETER LEGTH bit cao
					lidarFrame.parameterLength = (by << 8)
					trang_thai = 7
				case 7:
					#PARAMETER LEGTH bit thấp
					lidarFrame.parameterLength += byte
					lidarFrame.parameters.clear()
					trang_thai = 8
				case 8:
					#PARAMETER
					lidarFrame.parameters.append(byte)
					if len(lidarFrame.parameters) == lidarFrame.parameterLength:
						#Đọc đủ PARAMETER
						trang_thai = 9
				case 9:
					#CHECK CODE bit cao
					lidarFrame.check_code = (byte << 8)
					trang_thai = 10
				case 10:
					#CHECK CODE bit thấp
					lidarFrame.check_code += byte
					#Hoàn thành đọc khung truyền
					#Kiểm tra khung truyền có lỗi không
					if lidarFrame.check_code == check_code:
						#Khung truyền không lỗi
						LiDARFrameProcessing(lidarFrame)
					else:
						#Khung truyền lỗi đọc lại
						print("Khung truyền bị lỗi, tiến hành đọc lại");
					trang_thai = 0
			#Tính tổng các byte
			if trang_thai < 10:
				check_code = (check_code + byte) % 0xFFFF

if __name__ == "__main__":
	main()