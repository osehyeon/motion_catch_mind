using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using OpenCvSharp;
using OpenCvSharp.CPlusPlus;
using OpenCvSharp.Extensions;

//TCP/IP 통신을 위한  
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.IO;

namespace MotionProject
{
    public partial class From1 : Form
    {

        StreamReader streamreader1; // 데이터 읽기 위한 스트림리더
        StreamWriter streamwriter1; //  데이터 쓰기 위한 스트림라이터
        string answer;

        Mat Source;
        Mat re_Source;
        Mat Source_Gray;
        Mat Source_Binary;
        Mat Source_Test;
        Mat Source_Erode;
        Mat Source_Edge;
        Mat dst;

        OpenCvSharp.CPlusPlus.Point[][] contours;
        HierarchyIndex[] hierarchy;

        public int nServoOn = 0;
        public int nZBreak = 0;
        public int nLimit = 0;
        public int nAlarm = 0;
        public int nStop = 0;

        public From1()
        {
            InitializeComponent();
            if (CAXL.AxlOpen(7) == (uint)AXT_FUNC_RESULT.AXT_RT_SUCCESS)
                MessageBox.Show("AXL 라이브러리 초기화 성공");
            else
                MessageBox.Show("AXL 라이브러리 초기화 실패");

            // unit/pulse
            CAXM.AxmMotSetMoveUnitPerPulse(0, 10, 3000); // unit per pulse
            CAXM.AxmMotSetMoveUnitPerPulse(1, 10, 3000);
            CAXM.AxmMotSetMoveUnitPerPulse(2, 10, 3000);
            // pulse Mode
            CAXM.AxmMotSetPulseOutMethod(0, 6);  // 2 pulse ,CW, CCW, Active High
            CAXM.AxmMotSetPulseOutMethod(1, 6);
            CAXM.AxmMotSetPulseOutMethod(2, 6);
            // pulse output Mode
            CAXM.AxmMotSetEncInputMethod(0, 3);  // forward 4 
            CAXM.AxmMotSetEncInputMethod(1, 3);
            CAXM.AxmMotSetEncInputMethod(2, 3);
            // coordination, profile
            CAXM.AxmMotSetAbsRelMode(0, 0);   // axis, absolute 
            CAXM.AxmMotSetProfileMode(0, 3);  // axis, s curve
            CAXM.AxmMotSetAbsRelMode(1, 0);
            CAXM.AxmMotSetProfileMode(1, 3);
            CAXM.AxmMotSetAbsRelMode(2, 0);
            CAXM.AxmMotSetProfileMode(2, 3);
        }

        private void button_ServoOn_Click(object sender, EventArgs e)
        {
            
            if (nServoOn == 0)
            {
                CAXM.AxmSignalServoOn(0, 1);  // Servo On
                CAXM.AxmSignalServoOn(1, 1);  // Servo On
                CAXM.AxmSignalServoOn(2, 1);  // Servo On
                button_ServoOn.BackColor = Color.LightGreen;
            }
            else
            {
                CAXM.AxmSignalServoOn(0, 0);  // Servo Off
                CAXM.AxmSignalServoOn(1, 0);  // Servo On
                CAXM.AxmSignalServoOn(2, 0);  // Servo On
                button_ServoOn.BackColor = Color.LightGray;
            }
            nServoOn = ~nServoOn;
             
        }

        private void button_ZBreak_Click(object sender, EventArgs e)
        {
            if (nZBreak == 0)
            {
                CAXM.AxmSignalWriteOutputBit(2, 2, 1);  // 출력 On
                button_ZBreak.BackColor = Color.LightGreen;
            }
            else
            {
                CAXM.AxmSignalWriteOutputBit(2, 2, 0);  // 출력 Off
                button_ZBreak.BackColor = Color.LightGray;
            }
            nZBreak = ~nZBreak;
        }

        private void button_Home_Click(object sender, EventArgs e)
        {
            CAXM.AxmHomeSetMethod(0, 1, 4, 0, 1000, 0.0);  // Axis, CCW, ORG, Z(x), ms, place
            CAXM.AxmHomeSetMethod(1, 1, 4, 0, 1000, 0.0);
            CAXM.AxmHomeSetMethod(2, 1, 4, 0, 1000, 0.0);

            CAXM.AxmHomeSetVel(0, 100.0, 100.0, 20.0, 1.0, 400.0, 400.0);
            CAXM.AxmHomeSetVel(1, 100.0, 100.0, 20.0, 1.0, 400.0, 400.0);
            CAXM.AxmHomeSetVel(2, 100.0, 100.0, 20.0, 1.0, 400.0, 400.0);

            CAXM.AxmHomeSetStart(0);
            CAXM.AxmHomeSetStart(1);
            CAXM.AxmHomeSetStart(2);
        }

        private void button_LineMove_Click(object sender, EventArgs e)
        {
            int nAxisNo = Convert.ToInt32(comboBox_SelAxis.SelectedItem.ToString());
     
            int nVel = Convert.ToInt32(textBox_Vel.Text);

            int nAccel = Convert.ToInt32(textBox_Accel.Text);

            int nDecel = Convert.ToInt32(textBox_Decel.Text);

            int nMovePos = Convert.ToInt32(textBox_Pos.Text);

            CAXM.AxmMovePos(nAxisNo, nMovePos, nVel, nAccel, nDecel);
        }

        private void button_Stop_Click(object sender, EventArgs e)
        {
            
            CAXM.AxmMoveEStop(0);
            CAXM.AxmMoveEStop(1);
            CAXM.AxmMoveEStop(2);
                        
        }

        // ***** 그림 그리기 ***** //
        private void button_Draw_Click(object sender, EventArgs e)
        {
            int[] nPos = new int[2];
            int[] nAxis = new int[2] { 0, 1 };
            double[] fPos = new double[2];
            double[] fVel = new double[2] { 100, 100 };
            double[] fAcc = new double[2] { 100, 100 };
            double[] fDec = new double[2] { 100, 100 };
            
            Cv2.FindContours(Source_Erode, out contours, out hierarchy, ContourRetrieval.List, ContourChain.ApproxNone);

            int length = contours.Length;

            //Console.WriteLine("contours 사이즈 : " + length);

            CAXM.AxmMovePos(2, -90, 100, 100, 100);


            for (int i = 0; i < length; i++)
            {
            
                CAXM.AxmMovePos(2, -90, 100, 100, 100);
                if (contours[i].Length > 15)
                {
                    //Console.WriteLine(i + " : " + contours[i].Length);
                    //Console.Write(i + " 좌표 들 : ");
                    for (int j = 0; j < contours[i].Length; j++)
                    {
                        nPos[0] = contours[i][j].X;
                        nPos[1] = contours[i][j].Y - 500;
                        fPos[0] = Convert.ToDouble(nPos[0]);
                        fPos[1] = Convert.ToDouble(nPos[1]);
                        CAXM.AxmMoveMultiPos(2, nAxis, fPos, fVel, fAcc, fDec);
                        CAXM.AxmMovePos(2, -100, 100, 100, 100);
                        //Console.Write(contours[i][j]);
                    }
                }
            }

        }

        private void button_Limit_Click(object sender, EventArgs e)
        {
            if (nLimit == 0)
            {
                CAXM.AxmSignalSetLimit(0, 0, 1, 1);
                CAXM.AxmSignalSetLimit(1, 0, 1, 1);
                CAXM.AxmSignalSetLimit(2, 0, 1, 1);
                button_Limit.BackColor = Color.LightPink;
            }
            else
            {
                CAXM.AxmSignalSetLimit(0, 0, 0, 0);
                CAXM.AxmSignalSetLimit(1, 0, 0, 0);
                CAXM.AxmSignalSetLimit(2, 0, 0, 0);
                button_Limit.BackColor = Color.LightGreen;
            }
            nLimit = ~nLimit;
        }

        private void button_Alarm_Click(object sender, EventArgs e)
        {
            if (nAlarm == 0)
            {
                CAXM.AxmSignalSetServoAlarm(0, 1);
                CAXM.AxmSignalSetServoAlarm(1, 1);
                CAXM.AxmSignalSetServoAlarm(2, 1);
                button_Alarm.BackColor = Color.LightPink;
            }
            else
            {
                CAXM.AxmSignalSetServoAlarm(0, 0);
                CAXM.AxmSignalSetServoAlarm(1, 0);
                CAXM.AxmSignalSetServoAlarm(2, 0);
                button_Alarm.BackColor = Color.LightGreen;
            }
            nAlarm = ~nAlarm;
        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void button_Point_Click(object sender, EventArgs e)
        {
            CAXM.AxmMovePos(1, -300, 100, 100, 100);
            CAXM.AxmMovePos(2, -100, 100, 100, 100);
        }

        private void button_Test_Click(object sender, EventArgs e)
        {
            // coordination, profile
            CAXM.AxmMotSetAbsRelMode(0, 1);   // axis, absolute 
            CAXM.AxmMotSetProfileMode(0, 3);  // axis, s curve
            CAXM.AxmMotSetAbsRelMode(1, 1);
            CAXM.AxmMotSetProfileMode(1, 3);
            CAXM.AxmMotSetAbsRelMode(2, 1);
            CAXM.AxmMotSetProfileMode(2, 3);

            CAXM.AxmMoveStartPos(0, 100, 100, 100, 100);
            CAXM.AxmMoveStartPos(1, 100, 100, 100, 100);
            CAXM.AxmMoveStartPos(0, -100, 100, 100, 100);
            CAXM.AxmMoveStartPos(1, -100, 100, 100, 100);

            // coordination, profile
            CAXM.AxmMotSetAbsRelMode(0, 0);   // axis, absolute 
            CAXM.AxmMotSetProfileMode(0, 3);  // axis, s curve
            CAXM.AxmMotSetAbsRelMode(1, 0);
            CAXM.AxmMotSetProfileMode(1, 3);
            CAXM.AxmMotSetAbsRelMode(2, 0);
            CAXM.AxmMotSetProfileMode(2, 3);
        }

        private void button_pJogX_Click(object sender, EventArgs e)
        {
     
            CAXM.AxmMotSetAbsRelMode(0, 1);   
            CAXM.AxmMoveStartPos(0, 50, 100, 100, 100);
            CAXM.AxmMotSetAbsRelMode(0, 0);  

        }

        private void button_nJogX_Click(object sender, EventArgs e)
        {
            CAXM.AxmMotSetAbsRelMode(0, 1);
            CAXM.AxmMoveStartPos(0, -50, 100, 100, 100);
            CAXM.AxmMotSetAbsRelMode(0, 0);
        }

        private void button_pJogY_Click(object sender, EventArgs e)
        {
            CAXM.AxmMotSetAbsRelMode(1, 1);
            CAXM.AxmMoveStartPos(1, 50, 100, 100, 100);
            CAXM.AxmMotSetAbsRelMode(1, 0);
        }

        private void button_nJogY_Click(object sender, EventArgs e)
        {
            CAXM.AxmMotSetAbsRelMode(1, 1);
            CAXM.AxmMoveStartPos(1, -50, 100, 100, 100);
            CAXM.AxmMotSetAbsRelMode(1, 0);
        }

        private void button_pJogZ_Click(object sender, EventArgs e)
        {
            CAXM.AxmMotSetAbsRelMode(2, 1);
            CAXM.AxmMoveStartPos(2, 25, 100, 100, 100);
            CAXM.AxmMotSetAbsRelMode(2, 0);
        }

        private void button_nJogZ_Click(object sender, EventArgs e)
        {
            CAXM.AxmMotSetAbsRelMode(2, 1);
            CAXM.AxmMoveStartPos(2, -25, 100, 100, 100);
            CAXM.AxmMotSetAbsRelMode(2, 0);
        }

        private void button_Input_Click(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();

            openFileDialog1.InitialDirectory = "C:/.";
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                Source = new Mat(openFileDialog1.FileName, LoadMode.AnyColor);
                pictureBox_Input.SizeMode = PictureBoxSizeMode.Zoom;
                pictureBox_Input.Image = Source.ToBitmap();
                Cv2.WaitKey(0);
            }
        }

        private void pictureBox_Input_Click(object sender, EventArgs e)
        {
            re_Source = new Mat();
            Cv2.Resize(Source, re_Source, new OpenCvSharp.CPlusPlus.Size(Source.Width * 3, Source.Height * 3));
            Cv2.ImShow("Input Image", re_Source);
            Cv2.WaitKey(0);
            Cv2.DestroyAllWindows();
        }

        private void trackBar_Gray_Scroll(object sender, EventArgs e)
        {
            Mat element = Cv2.GetStructuringElement(StructuringElementShape.Rect, new OpenCvSharp.CPlusPlus.Size(6, 6));

            trackBar_Gray.Maximum = 255;
            trackBar_Gray.Minimum = 0;
            int threshold_val = trackBar_Gray.Value;
            textBox_Gray.TextAlign = HorizontalAlignment.Center;
            textBox_Gray.Text = (Convert.ToString(threshold_val));
            Source_Gray = new Mat();
            Source_Binary = new Mat();
            Source_Erode = new Mat();
            dst = new Mat();
            Cv2.CvtColor(Source, Source_Gray, ColorConversion.BgrToGray);
            Cv2.CvtColor(Source_Gray, dst, ColorConversion.GrayToBgr);
            Cv2.Threshold(Source_Gray, Source_Erode, threshold_val, 255, ThresholdType.Binary); //ToZero
            //Cv2.Erode(Source_Binary, Source_Erode, element, new OpenCvSharp.CPlusPlus.Point(3,3), 1);
            pictureBox_Gray.SizeMode = PictureBoxSizeMode.Zoom;
            pictureBox_Gray.Image = Source_Erode.ToBitmap();
            Cv2.WaitKey(0);
        }

        private void pictureBox_Gray_Click(object sender, EventArgs e)
        {
            re_Source = new Mat();
            Cv2.Resize(Source_Erode, re_Source, new OpenCvSharp.CPlusPlus.Size(Source_Erode.Width * 3, Source_Erode.Height * 3));
            Cv2.ImShow("Gray", re_Source);
            Cv2.WaitKey(0);
            Cv2.DestroyAllWindows();
        }
        
        private void button_Convert_Click(object sender, EventArgs e)
        {
            Cv2.FindContours(Source_Erode, out contours, out hierarchy, ContourRetrieval.List, ContourChain.ApproxNone);
            int length = contours.Length;

            
            for (int i = 0; i < length ; i++)
            {
                if (contours[i].Length > 15)
                {
                    Console.WriteLine(i + " : " + contours[i].Length);

                    Cv2.DrawContours(dst, contours, i, Scalar.Red, 2);
                }
     
                
            }
            pictureBox_Contours.SizeMode = PictureBoxSizeMode.Zoom;
            pictureBox_Contours.Image = dst.ToBitmap();
            Cv2.WaitKey(0);
        }

        private void sendbutton_Click(object sender, EventArgs e)
        {
            string sendData1 = textBox3.Text; //textBox3 의 내용을 sendData1 변수에 저장
            streamwriter1.WriteLine(sendData1); //스트림라이터를 통해 데이터를 전송
        }

        private void ConnectButton_Click(object sender, EventArgs e)
        {
            Thread thread1 = new Thread(connect); //Therad 객체 생성, Form 과는 별도 쓰레드에서 connect 함수가 실행된
            thread1.IsBackground = true; //Form 이 종료되면 thread1 도 종료
            thread1.Start(); //thread1 시작
        }

        private void connect() //thread1 에 연결된 함수, 메인 폼과는 별도로 동작한다.
        {
            TcpListener tcpListener1 = new TcpListener(IPAddress.Parse(IpBox.Text), int.Parse(PortBox.Text)); //서버 객체 생성 및 IP 주소와 Port 번호를 할당
            tcpListener1.Start(); // 서버 시작
            writeRichTextbox("서버 준비... 클라이언트 기다리는 중...");

            TcpClient tcpClient1 = tcpListener1.AcceptTcpClient(); //클라이언트 접속 확인
            writeRichTextbox("클라이언트 연결됨...");

            streamreader1 = new StreamReader(tcpClient1.GetStream()); ///읽기 스트림 연결
            streamwriter1 = new StreamWriter(tcpClient1.GetStream()); //쓰기 스트림 연결
            streamwriter1.AutoFlush = true; //쓰기 버퍼 자동으로 뭔가 처리...

            while (tcpClient1.Connected)
            {
                string receiveData1 = streamreader1.ReadLine(); //수신 데이터를 읽어서 receiveData1 변수에 저장
                writeRichTextbox(receiveData1);
                if (receiveData1 == answer)
                    streamwriter1.WriteLine("정답");
                else streamwriter1.WriteLine("다시 생각해보렴");

            }

        }

        private void writeRichTextbox(string str)
        {
            richTextBox1.Invoke((MethodInvoker)delegate { richTextBox1.AppendText(str + "\r\n"); }); // 데이터를 수신창에 표시, 반드시 Invoke 사용, 충돌피함
            richTextBox1.Invoke((MethodInvoker)delegate { richTextBox1.ScrollToCaret(); }); // 스크롤을 젤 밑으로
        }

        private void answerbutton_Click(object sender, EventArgs e)
        {
            answer = answerbox.Text;
            writeRichTextbox("문제 출제 완료");
        }


    }
}
