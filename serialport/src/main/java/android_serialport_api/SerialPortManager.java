package android_serialport_api;

import android.util.Log;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * @author woong
 * Created by woong on 2018/3/14
 */

public class SerialPortManager {
    private final String TAG = "SerialPortManager";
    private final SerialDevice serialDevice;
    /**
     * 是否打开串口标志
     */
    private final AtomicBoolean mSerialPortStatus = new AtomicBoolean(false);
    /**
     * 线程状态，为了安全终止线程
     */
    private final AtomicBoolean mThreadStatus = new AtomicBoolean(false);

    public SerialPort mSerialPort = null;
    public InputStream mInputStream = null;
    public OutputStream mOutputStream = null;
    // 定义每次发送的最大字节数（可根据实际情况调整）
    private static final int MAX_PACKET_SIZE = 256;
    // 定义发送间隔（毫秒），给串口足够的处理时间
    private static final int SEND_INTERVAL = 30;

    private ExecutorService sendExecutor;

    public SerialPortManager(SerialDevice serialDevice) {
        openSerialPort(serialDevice);
        this.serialDevice = serialDevice;
        sendExecutor = Executors.newSingleThreadExecutor();
    }

    /**
     * 打开串口
     *
     * @return serialPort串口对象
     */
    private SerialPort openSerialPort(SerialDevice serialDevice) {
        try {
            File deviceFile = new File(serialDevice.path);
            if (!deviceFile.exists()) {
                Log.e(TAG, "serialDevice is null == ");
                return null;
            }
            mSerialPort = new SerialPort(serialDevice);

            mSerialPortStatus.set(true);
            mThreadStatus.set(false);

            mInputStream = mSerialPort.getInputStream();
            mOutputStream = mSerialPort.getOutputStream();

            //开始线程监控是否有数据要接收
            ReadThread mReadThread = new ReadThread();
            mReadThread.setName("Recv Thread");
            mReadThread.start();
        } catch (IOException e) {
            Log.e(TAG, "openSerialPort == : 打开串口异常：" + e.toString());
            e.printStackTrace();
            return mSerialPort;
        }
        Log.i(TAG, "openSerialPort == : 打开串口成功");
        return mSerialPort;
    }

    /**
     * 关闭串口
     */
    public void closeSerialPort() {
        try {
            if (mSerialPortStatus.get()) {
                mSerialPortStatus.set(false);
                mThreadStatus.set(true);

                mInputStream.close();
                mOutputStream.close();

                mSerialPort.close();
                Log.i(TAG, "closeSerialPort == : 关闭串口成功");
            }
            if (sendExecutor != null) {
                sendExecutor.shutdown();
                sendExecutor = null;
            }
        } catch (IOException e) {
            Log.e(TAG, "closeSerialPort == : 关闭串口异常：" + e.toString());
        }
    }

    private long calculateTransmissionTime(int dataLength) {
        // 数据位 + 起始位 + 停止位（通常每字节10位）
        long totalBits = (long) dataLength * 12;
        // 计算传输时间（毫秒），向上取整确保足够时间
        return (long) Math.ceil(totalBits * 1000.0 / serialDevice.speed);
    }

    public void sendPacket(byte[] sendData) {
        if (sendData == null || sendData.length == 0 || !mSerialPortStatus.get()) {
            return;
        }
        try {
            int offset = 0;
            int remaining = sendData.length;

            while (remaining > 0 && !Thread.currentThread().isInterrupted()) {
                int length = Math.min(remaining, MAX_PACKET_SIZE);
                mOutputStream.write(sendData, offset, length);
                mOutputStream.flush();

                offset += length;
                remaining -= length;
                Thread.sleep(calculateTransmissionTime(length));
            }
        } catch (IOException e) {
            e.getMessage();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


    public void sendPacket0(byte[] sendData) {
        if (sendData == null || sendData.length == 0 || !mSerialPortStatus.get()) {
            return;
        }
        if (sendExecutor == null || sendExecutor.isShutdown()) {
            return;
        }
        sendExecutor.execute(() -> {
            try {
                int offset = 0;
                int remaining = sendData.length;

                while (remaining > 0 && !Thread.currentThread().isInterrupted()) {
                    int length = Math.min(remaining, MAX_PACKET_SIZE);
                    mOutputStream.write(sendData, offset, length);
                    mOutputStream.flush();

                    offset += length;
                    remaining -= length;

                    if (remaining > 0) {
                        Thread.sleep(serialDevice.speed == 115200 ? SEND_INTERVAL : SEND_INTERVAL * 10);
                    }
                }
            } catch (IOException e) {
                e.getMessage();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        });
    }


    /**
     * 数据接收线程
     */
    private class ReadThread extends Thread {
        // 缓冲区大小，可以根据实际需求调整
        private static final int BUFFER_SIZE = 1024;

        @Override
        public void run() {
            super.run();
            // 为每个线程创建独立的缓冲区，避免多线程竞争
            byte[] buffer = new byte[BUFFER_SIZE];
            // 判断线程是否在运行，更安全的结束线程
            while (!mThreadStatus.get() && !isInterrupted()) {
                try {
                    // 检查输入流是否有效
                    if (mInputStream == null) {
                        Log.e(TAG, "输入流为空，退出接收线程");
                        return;
                    }

                    // 读取数据
                    int available = mInputStream.available();
                    if (available > 0) {
                        // 确保缓冲区足够大
                        int size = Math.min(available, BUFFER_SIZE);
                        int bytesRead = mInputStream.read(buffer, 0, size);

                        if (bytesRead > 0) {
                            // 复制有效数据
                            byte[] readBytes = new byte[bytesRead];
                            System.arraycopy(buffer, 0, readBytes, 0, bytesRead);

                            // 回调数据接收事件
                            if (onDataReceiveListener != null) {
                                onDataReceiveListener.onDataReceive(readBytes, bytesRead);
                            }
                        }
                    } else {
                        Thread.sleep(10);
                    }
                } catch (IOException e) {
                    //Log.e(TAG, "run == : 数据读取异常：" + e.toString());
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    /**
     * 监听数据接收
     */
    public OnDataReceiveListener onDataReceiveListener = null;

    public interface OnDataReceiveListener {
        void onDataReceive(byte[] buffer, int size);
    }

    public void setOnDataReceiveListener(OnDataReceiveListener dataReceiveListener) {
        onDataReceiveListener = dataReceiveListener;
    }
}
