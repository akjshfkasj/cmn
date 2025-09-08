package android_serialport_api;

/**
 * @author Woong on 12/4/20
 * @website http://woong.cn
 */
public class SerialDevice {
    public String path;
    public int speed = 115200;
    public int dataBits = 8;
    public int stopBits = 1;
    public char parity = 'n'; // n: 无检验 o: 奇校验 e: 偶检验
    public boolean block = false;

    public SerialDevice() {
    }

    public SerialDevice(String path, int speed, int dataBits, int stopBits, char parity, boolean block) {

        this.path = path;
        this.speed = speed;
        this.dataBits = dataBits;
        this.stopBits = stopBits;
        this.parity = parity;
        this.block = block;
    }
}
