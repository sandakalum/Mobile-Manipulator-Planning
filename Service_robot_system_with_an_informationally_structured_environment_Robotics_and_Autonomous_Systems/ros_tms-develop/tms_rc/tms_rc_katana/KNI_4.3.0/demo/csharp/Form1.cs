using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Runtime.InteropServices;




namespace csharp
{
    public partial class Form1 : Form
    {
        [StructLayout(LayoutKind.Sequential)]
        public struct TPos
        {
            public double X, Y, Z, Phi, Theta, Psi;

        }
        
        
        [DllImport("../../../../lib/win32/KNI_Wrapper.dll")]
        static extern int initKatana(string configFile, string ipAddress);
        //[DllImport("KNI_Wrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        [DllImport("../../../../lib/win32/KNI_Wrapper.dll")]
        static extern int calibrate(int axis);
        [DllImport("../../../../lib/win32/KNI_Wrapper.dll")]
        static extern int getPosition(ref TPos position);
        [DllImport("../../../../lib/win32/KNI_Wrapper.dll")]
        static extern int moveToPos(ref TPos position, int vel, int accel);
        [DllImport("../../../../lib/win32/KNI_Wrapper.dll")]
        static extern int moveToPosLin(ref TPos position, int vel, int accel);
        [DllImport("../../../../lib/win32/KNI_Wrapper.dll")]
        static extern int allMotorsOn();
        [DllImport("../../../../lib/win32/KNI_Wrapper.dll")]
        static extern int allMotorsOff();


        public Form1()
        {
            InitializeComponent();
        }
        
        private void initialize_Click(object sender, EventArgs e)
        {
            
             //use Socket connection:
             tbOutput.AppendText("Initializing Katana...");
             //katana = new KNInet.Katana(tbIpAddress.Text, comPort.Text, configurationFile.Text);
             if (initKatana(toolStripConfigFile.Text, toolStripIPAddress.Text) == 1)
             {
                 tbOutput.AppendText(" done.\n");
                 BtnCalibrate.Enabled = true;
                 MotorsOff.Enabled = true;
                 MotorsOn.Enabled = true;
                 buttonRead.Enabled = true;
                 toolStripConnect.Image = csharp.Properties.Resources.connect;
                 //return ((System.Drawing.Bitmap)(obj));

             }
             else
                 tbOutput.AppendText(" failed.\n");

          
        }
        private void configurationFile_Click(object sender, EventArgs e)
        {
            openConfigFileDialog.ShowDialog();
        }
        private void calibrate_Click(object sender, EventArgs e)
        {
            tbOutput.AppendText("Calibrating Katana...");
            calibrate(0);
            tbOutput.AppendText(" done\n");
            buttonGo.Enabled = true;
            buttonRead_Click(sender, e);
            
        }

        private void openConfigFileDialog_FileOk(object sender, CancelEventArgs e)
        {
            toolStripConfigFile.Text = openConfigFileDialog.FileName;
        }

        private void run_Click(object sender, EventArgs e)
        {
            tbOutput.AppendText("Demo:\n");

        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void configurationFile_TextChanged(object sender, EventArgs e)
        {

        }

        private void btnClose_Click(object sender, EventArgs e)
        {
            Close();
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void label1_Click(object sender, EventArgs e)
        {

        }

        private void groupBox1_Enter(object sender, EventArgs e)
        {

        }

        private void buttonRead_Click(object sender, EventArgs e)
        {
            tbOutput.AppendText("Read Coordinates.\n");
            TPos position = new TPos();
            getPosition(ref position);
            numericUpDownX.Value =(decimal)position.X;
            numericUpDownY.Value = (decimal)position.Y;
            numericUpDownZ.Value = (decimal)position.Z;
            numericUpDownPhi.Value = (decimal)position.Phi;
            numericUpDownTheta.Value = (decimal)position.Theta;
            numericUpDownPsi.Value = (decimal)position.Psi;

        }

        private void buttonGo_Click(object sender, EventArgs e)
        {
            TPos position = new TPos();
            position.X = (double)numericUpDownX.Value;
            position.Y = (double)numericUpDownY.Value;
            position.Z = (double)numericUpDownZ.Value;
            position.Phi = (double)numericUpDownPhi.Value;
            position.Theta = (double)numericUpDownTheta.Value;
            position.Psi = (double)numericUpDownPsi.Value;
            if (checkBoxLM.Checked == true)
            {
                tbOutput.AppendText("Move Linear.. ");
                moveToPosLin(ref position, (int)numericUpDownVel.Value, 1);
                tbOutput.AppendText("Done.\n");
            }
            else
            {
                tbOutput.AppendText("Move PTP.. ");
                moveToPos(ref position, (int)numericUpDownVel.Value, 1);
                tbOutput.AppendText("Done.\n");
            }
        }

        private void numericUpDownX_ValueChanged(object sender, EventArgs e)
        {

        }

        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void label7_Click(object sender, EventArgs e)
        {

        }

        private void tbIpAddress_TextChanged(object sender, EventArgs e)
        {

        }

        private void toolStripIPAddress_Click(object sender, EventArgs e)
        {

        }

        private void tbOutput_TextChanged(object sender, EventArgs e)
        {

        }

        private void MotorsOn_Click(object sender, EventArgs e)
        {
            tbOutput.AppendText("Motors ON.\n");
            allMotorsOn();
            buttonRead_Click(sender, e);
        }

        private void MotorsOff_Click(object sender, EventArgs e)
        {
            tbOutput.AppendText("Motors OFF.\n");
            allMotorsOff();
        }

        private void pictureBox1_Click(object sender, EventArgs e)
        {

        }

    }
}