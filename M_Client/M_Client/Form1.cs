using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
//TCPIP
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.IO;

namespace M_Client
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        StreamReader streamReader; //데이터 읽기 위한 스트림리더
        StreamWriter streamWriter; //데이터 쓰기 위한 스트림라이더


        private void connect()  // thread1에 연결된 함수. 메인폼과는 별도로 동작한다.
        {
            TcpClient tcpClient1 = new TcpClient();  // TcpClient 객체 생성
            IPEndPoint ipEnd = new IPEndPoint(IPAddress.Parse(textBox1.Text), int.Parse(textBox2.Text));  // IP주소와 Port번호를 할당
            tcpClient1.Connect(ipEnd);  // 서버에 연결 요청
            writeRichTextbox("서버 연결됨...");

            streamReader = new StreamReader(tcpClient1.GetStream());  // 읽기 스트림 연결
            streamWriter = new StreamWriter(tcpClient1.GetStream());  // 쓰기 스트림 연결
            streamWriter.AutoFlush = true;  // 쓰기 버퍼 자동으로 뭔가 처리..

            while (tcpClient1.Connected)  // 클라이언트가 연결되어 있는 동안
            {
                string receiveData1 = streamReader.ReadLine();  // 수신 데이타를 읽어서 receiveData1 변수에 저장
                writeRichTextbox(receiveData1);  // 데이타를 수신창에 쓰기
            }
           
        }

        private void writeRichTextbox(string data)  // richTextbox1 에 쓰기 함수
        {
            richTextBox1.Invoke((MethodInvoker)delegate { richTextBox1.AppendText(data + "\r\n"); });   //  데이타를 수신창에 표시, 반드시 invoke 사용. 충돌피함.
            richTextBox1.Invoke((MethodInvoker)delegate { richTextBox1.ScrollToCaret(); });             // 스크롤을 젤 밑으로.
        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            Thread thread1 = new Thread(connect);   // Thread 객채 생성, Form과는 별도 쓰레드에서 connect 함수가 실행됨.
            thread1.IsBackground = true;            // Form이 종료되면 thread1도 종료.
            thread1.Start();                        // thread1 시작.
        }

        private void button2_Click_1(object sender, EventArgs e)
        {
            string sendData1 = textBox3.Text;    // testBox3 의 내용을 sendData1 변수에 저장
            streamWriter.WriteLine(sendData1);   // 스트림라이터를 통해 데이타를 전송
        }
    }
}
