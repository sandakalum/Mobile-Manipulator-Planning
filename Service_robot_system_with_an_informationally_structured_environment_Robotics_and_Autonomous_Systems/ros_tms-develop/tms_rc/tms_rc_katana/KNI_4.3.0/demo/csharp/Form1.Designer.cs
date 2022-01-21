namespace csharp
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.BtnCalibrate = new System.Windows.Forms.Button();
            this.openConfigFileDialog = new System.Windows.Forms.OpenFileDialog();
            this.tbOutput = new System.Windows.Forms.TextBox();
            this.btnClose = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.groupBoxPosition = new System.Windows.Forms.GroupBox();
            this.label7 = new System.Windows.Forms.Label();
            this.checkBoxLM = new System.Windows.Forms.CheckBox();
            this.numericUpDownPsi = new System.Windows.Forms.NumericUpDown();
            this.numericUpDownTheta = new System.Windows.Forms.NumericUpDown();
            this.numericUpDownPhi = new System.Windows.Forms.NumericUpDown();
            this.numericUpDownZ = new System.Windows.Forms.NumericUpDown();
            this.numericUpDownY = new System.Windows.Forms.NumericUpDown();
            this.numericUpDownVel = new System.Windows.Forms.NumericUpDown();
            this.numericUpDownX = new System.Windows.Forms.NumericUpDown();
            this.buttonGo = new System.Windows.Forms.Button();
            this.buttonRead = new System.Windows.Forms.Button();
            this.toolStripSeparator4 = new System.Windows.Forms.ToolStripSeparator();
            this.toolStripLabel1 = new System.Windows.Forms.ToolStripLabel();
            this.toolStripIPAddress = new System.Windows.Forms.ToolStripTextBox();
            this.toolStripSeparator3 = new System.Windows.Forms.ToolStripSeparator();
            this.toolStripLabel2 = new System.Windows.Forms.ToolStripLabel();
            this.toolStripConfigFile = new System.Windows.Forms.ToolStripTextBox();
            this.toolStrip2 = new System.Windows.Forms.ToolStrip();
            this.toolStripConnect = new System.Windows.Forms.ToolStripButton();
            this.MotorsOn = new System.Windows.Forms.Button();
            this.MotorsOff = new System.Windows.Forms.Button();
            this.groupBoxPosition.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownPsi)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownTheta)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownPhi)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownZ)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownY)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownVel)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownX)).BeginInit();
            this.toolStrip2.SuspendLayout();
            this.SuspendLayout();
            // 
            // BtnCalibrate
            // 
            this.BtnCalibrate.Enabled = false;
            this.BtnCalibrate.Location = new System.Drawing.Point(295, 60);
            this.BtnCalibrate.Name = "BtnCalibrate";
            this.BtnCalibrate.Size = new System.Drawing.Size(75, 23);
            this.BtnCalibrate.TabIndex = 5;
            this.BtnCalibrate.Text = "Calibrate";
            this.BtnCalibrate.UseVisualStyleBackColor = true;
            this.BtnCalibrate.Click += new System.EventHandler(this.calibrate_Click);
            // 
            // openConfigFileDialog
            // 
            this.openConfigFileDialog.FileName = "openFileDialog1";
            this.openConfigFileDialog.Filter = "Configuration-Files (*.cfg)|*.cfg";
            this.openConfigFileDialog.InitialDirectory = "Environment.CurrentDirectory";
            this.openConfigFileDialog.Title = "Select configuration-file";
            this.openConfigFileDialog.FileOk += new System.ComponentModel.CancelEventHandler(this.openConfigFileDialog_FileOk);
            // 
            // tbOutput
            // 
            this.tbOutput.Location = new System.Drawing.Point(402, 60);
            this.tbOutput.Multiline = true;
            this.tbOutput.Name = "tbOutput";
            this.tbOutput.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.tbOutput.Size = new System.Drawing.Size(256, 213);
            this.tbOutput.TabIndex = 9;
            this.tbOutput.TextChanged += new System.EventHandler(this.tbOutput_TextChanged);
            // 
            // btnClose
            // 
            this.btnClose.Location = new System.Drawing.Point(570, 295);
            this.btnClose.Name = "btnClose";
            this.btnClose.Size = new System.Drawing.Size(88, 23);
            this.btnClose.TabIndex = 10;
            this.btnClose.Text = "Exit";
            this.btnClose.UseVisualStyleBackColor = true;
            this.btnClose.Click += new System.EventHandler(this.btnClose_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(134, 34);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(25, 13);
            this.label1.TabIndex = 12;
            this.label1.Text = "Phi:";
            this.label1.Click += new System.EventHandler(this.label1_Click);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(121, 60);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(38, 13);
            this.label5.TabIndex = 12;
            this.label5.Text = "Theta:";
            this.label5.Click += new System.EventHandler(this.label1_Click);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(135, 85);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(24, 13);
            this.label6.TabIndex = 12;
            this.label6.Text = "Psi:";
            this.label6.Click += new System.EventHandler(this.label1_Click);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(11, 34);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(17, 13);
            this.label3.TabIndex = 12;
            this.label3.Text = "X:";
            this.label3.Click += new System.EventHandler(this.label1_Click);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(11, 85);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(17, 13);
            this.label4.TabIndex = 12;
            this.label4.Text = "Z:";
            this.label4.Click += new System.EventHandler(this.label1_Click);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(11, 60);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(17, 13);
            this.label2.TabIndex = 12;
            this.label2.Text = "Y:";
            this.label2.Click += new System.EventHandler(this.label1_Click);
            // 
            // groupBoxPosition
            // 
            this.groupBoxPosition.Controls.Add(this.label7);
            this.groupBoxPosition.Controls.Add(this.checkBoxLM);
            this.groupBoxPosition.Controls.Add(this.numericUpDownPsi);
            this.groupBoxPosition.Controls.Add(this.numericUpDownTheta);
            this.groupBoxPosition.Controls.Add(this.numericUpDownPhi);
            this.groupBoxPosition.Controls.Add(this.numericUpDownZ);
            this.groupBoxPosition.Controls.Add(this.numericUpDownY);
            this.groupBoxPosition.Controls.Add(this.numericUpDownVel);
            this.groupBoxPosition.Controls.Add(this.numericUpDownX);
            this.groupBoxPosition.Controls.Add(this.buttonGo);
            this.groupBoxPosition.Controls.Add(this.buttonRead);
            this.groupBoxPosition.Controls.Add(this.label2);
            this.groupBoxPosition.Controls.Add(this.label4);
            this.groupBoxPosition.Controls.Add(this.label3);
            this.groupBoxPosition.Controls.Add(this.label6);
            this.groupBoxPosition.Controls.Add(this.label5);
            this.groupBoxPosition.Controls.Add(this.label1);
            this.groupBoxPosition.Location = new System.Drawing.Point(19, 54);
            this.groupBoxPosition.Name = "groupBoxPosition";
            this.groupBoxPosition.Size = new System.Drawing.Size(253, 213);
            this.groupBoxPosition.TabIndex = 13;
            this.groupBoxPosition.TabStop = false;
            this.groupBoxPosition.Text = "Coordinates";
            this.groupBoxPosition.Enter += new System.EventHandler(this.groupBox1_Enter);
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(85, 154);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(38, 13);
            this.label7.TabIndex = 17;
            this.label7.Text = "Speed";
            this.label7.Click += new System.EventHandler(this.label7_Click);
            // 
            // checkBoxLM
            // 
            this.checkBoxLM.AutoSize = true;
            this.checkBoxLM.Location = new System.Drawing.Point(34, 179);
            this.checkBoxLM.Name = "checkBoxLM";
            this.checkBoxLM.Size = new System.Drawing.Size(108, 17);
            this.checkBoxLM.TabIndex = 16;
            this.checkBoxLM.Text = "Linear Movement";
            this.checkBoxLM.UseVisualStyleBackColor = true;
            this.checkBoxLM.CheckedChanged += new System.EventHandler(this.checkBox1_CheckedChanged);
            // 
            // numericUpDownPsi
            // 
            this.numericUpDownPsi.DecimalPlaces = 2;
            this.numericUpDownPsi.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericUpDownPsi.Location = new System.Drawing.Point(165, 78);
            this.numericUpDownPsi.Minimum = new decimal(new int[] {
            100,
            0,
            0,
            -2147483648});
            this.numericUpDownPsi.Name = "numericUpDownPsi";
            this.numericUpDownPsi.Size = new System.Drawing.Size(63, 20);
            this.numericUpDownPsi.TabIndex = 14;
            // 
            // numericUpDownTheta
            // 
            this.numericUpDownTheta.DecimalPlaces = 2;
            this.numericUpDownTheta.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericUpDownTheta.Location = new System.Drawing.Point(165, 53);
            this.numericUpDownTheta.Minimum = new decimal(new int[] {
            100,
            0,
            0,
            -2147483648});
            this.numericUpDownTheta.Name = "numericUpDownTheta";
            this.numericUpDownTheta.Size = new System.Drawing.Size(63, 20);
            this.numericUpDownTheta.TabIndex = 14;
            // 
            // numericUpDownPhi
            // 
            this.numericUpDownPhi.DecimalPlaces = 2;
            this.numericUpDownPhi.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericUpDownPhi.Location = new System.Drawing.Point(165, 27);
            this.numericUpDownPhi.Minimum = new decimal(new int[] {
            100,
            0,
            0,
            -2147483648});
            this.numericUpDownPhi.Name = "numericUpDownPhi";
            this.numericUpDownPhi.Size = new System.Drawing.Size(63, 20);
            this.numericUpDownPhi.TabIndex = 14;
            // 
            // numericUpDownZ
            // 
            this.numericUpDownZ.DecimalPlaces = 1;
            this.numericUpDownZ.Location = new System.Drawing.Point(34, 78);
            this.numericUpDownZ.Maximum = new decimal(new int[] {
            500,
            0,
            0,
            0});
            this.numericUpDownZ.Minimum = new decimal(new int[] {
            500,
            0,
            0,
            -2147483648});
            this.numericUpDownZ.Name = "numericUpDownZ";
            this.numericUpDownZ.Size = new System.Drawing.Size(63, 20);
            this.numericUpDownZ.TabIndex = 14;
            // 
            // numericUpDownY
            // 
            this.numericUpDownY.DecimalPlaces = 1;
            this.numericUpDownY.Location = new System.Drawing.Point(34, 53);
            this.numericUpDownY.Maximum = new decimal(new int[] {
            500,
            0,
            0,
            0});
            this.numericUpDownY.Minimum = new decimal(new int[] {
            500,
            0,
            0,
            -2147483648});
            this.numericUpDownY.Name = "numericUpDownY";
            this.numericUpDownY.Size = new System.Drawing.Size(63, 20);
            this.numericUpDownY.TabIndex = 14;
            // 
            // numericUpDownVel
            // 
            this.numericUpDownVel.Location = new System.Drawing.Point(34, 150);
            this.numericUpDownVel.Maximum = new decimal(new int[] {
            180,
            0,
            0,
            0});
            this.numericUpDownVel.Minimum = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.numericUpDownVel.Name = "numericUpDownVel";
            this.numericUpDownVel.Size = new System.Drawing.Size(45, 20);
            this.numericUpDownVel.TabIndex = 14;
            this.numericUpDownVel.Value = new decimal(new int[] {
            80,
            0,
            0,
            0});
            this.numericUpDownVel.ValueChanged += new System.EventHandler(this.numericUpDownX_ValueChanged);
            // 
            // numericUpDownX
            // 
            this.numericUpDownX.DecimalPlaces = 1;
            this.numericUpDownX.Location = new System.Drawing.Point(34, 27);
            this.numericUpDownX.Maximum = new decimal(new int[] {
            500,
            0,
            0,
            0});
            this.numericUpDownX.Minimum = new decimal(new int[] {
            500,
            0,
            0,
            -2147483648});
            this.numericUpDownX.Name = "numericUpDownX";
            this.numericUpDownX.Size = new System.Drawing.Size(63, 20);
            this.numericUpDownX.TabIndex = 14;
            this.numericUpDownX.ValueChanged += new System.EventHandler(this.numericUpDownX_ValueChanged);
            // 
            // buttonGo
            // 
            this.buttonGo.Enabled = false;
            this.buttonGo.Location = new System.Drawing.Point(155, 150);
            this.buttonGo.Name = "buttonGo";
            this.buttonGo.Size = new System.Drawing.Size(73, 20);
            this.buttonGo.TabIndex = 13;
            this.buttonGo.Text = "Go";
            this.buttonGo.UseVisualStyleBackColor = true;
            this.buttonGo.Click += new System.EventHandler(this.buttonGo_Click);
            // 
            // buttonRead
            // 
            this.buttonRead.Enabled = false;
            this.buttonRead.Location = new System.Drawing.Point(155, 176);
            this.buttonRead.Name = "buttonRead";
            this.buttonRead.Size = new System.Drawing.Size(73, 20);
            this.buttonRead.TabIndex = 13;
            this.buttonRead.Text = "Read";
            this.buttonRead.UseVisualStyleBackColor = true;
            this.buttonRead.Click += new System.EventHandler(this.buttonRead_Click);
            // 
            // toolStripSeparator4
            // 
            this.toolStripSeparator4.Name = "toolStripSeparator4";
            this.toolStripSeparator4.Size = new System.Drawing.Size(6, 25);
            // 
            // toolStripLabel1
            // 
            this.toolStripLabel1.Name = "toolStripLabel1";
            this.toolStripLabel1.Size = new System.Drawing.Size(65, 22);
            this.toolStripLabel1.Text = "IP Address:";
            // 
            // toolStripIPAddress
            // 
            this.toolStripIPAddress.Name = "toolStripIPAddress";
            this.toolStripIPAddress.Size = new System.Drawing.Size(90, 25);
            this.toolStripIPAddress.Text = "192.168.1.1";
            this.toolStripIPAddress.Click += new System.EventHandler(this.toolStripIPAddress_Click);
            // 
            // toolStripSeparator3
            // 
            this.toolStripSeparator3.Name = "toolStripSeparator3";
            this.toolStripSeparator3.Size = new System.Drawing.Size(6, 25);
            // 
            // toolStripLabel2
            // 
            this.toolStripLabel2.Name = "toolStripLabel2";
            this.toolStripLabel2.Size = new System.Drawing.Size(105, 22);
            this.toolStripLabel2.Text = "Configuration File:";
            // 
            // toolStripConfigFile
            // 
            this.toolStripConfigFile.Name = "toolStripConfigFile";
            this.toolStripConfigFile.Size = new System.Drawing.Size(340, 35);
            this.toolStripConfigFile.Text = "click to open";
            this.toolStripConfigFile.Click += new System.EventHandler(this.configurationFile_Click);
            // 
            // toolStrip2
            // 
            this.toolStrip2.AutoSize = false;
            this.toolStrip2.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStripConnect,
            this.toolStripSeparator4,
            this.toolStripLabel1,
            this.toolStripIPAddress,
            this.toolStripSeparator3,
            this.toolStripLabel2,
            this.toolStripConfigFile});
            this.toolStrip2.Location = new System.Drawing.Point(0, 0);
            this.toolStrip2.Name = "toolStrip2";
            this.toolStrip2.Size = new System.Drawing.Size(669, 35);
            this.toolStrip2.TabIndex = 16;
            this.toolStrip2.Text = "toolStrip2";
            // 
            // toolStripConnect
            // 
            this.toolStripConnect.AutoSize = false;
            this.toolStripConnect.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Image;
            this.toolStripConnect.Image = global::csharp.Properties.Resources.connect_no;
            this.toolStripConnect.ImageScaling = System.Windows.Forms.ToolStripItemImageScaling.None;
            this.toolStripConnect.ImageTransparentColor = System.Drawing.Color.Magenta;
            this.toolStripConnect.Name = "toolStripConnect";
            this.toolStripConnect.Size = new System.Drawing.Size(32, 32);
            this.toolStripConnect.Text = "Initialize";
            this.toolStripConnect.Click += new System.EventHandler(this.initialize_Click);
            // 
            // MotorsOn
            // 
            this.MotorsOn.Enabled = false;
            this.MotorsOn.Location = new System.Drawing.Point(295, 101);
            this.MotorsOn.Name = "MotorsOn";
            this.MotorsOn.Size = new System.Drawing.Size(75, 23);
            this.MotorsOn.TabIndex = 17;
            this.MotorsOn.Text = "Motors On";
            this.MotorsOn.UseVisualStyleBackColor = true;
            this.MotorsOn.Click += new System.EventHandler(this.MotorsOn_Click);
            // 
            // MotorsOff
            // 
            this.MotorsOff.Enabled = false;
            this.MotorsOff.Location = new System.Drawing.Point(295, 129);
            this.MotorsOff.Name = "MotorsOff";
            this.MotorsOff.Size = new System.Drawing.Size(75, 23);
            this.MotorsOff.TabIndex = 18;
            this.MotorsOff.Text = "Motors Off";
            this.MotorsOff.UseVisualStyleBackColor = true;
            this.MotorsOff.Click += new System.EventHandler(this.MotorsOff_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.Window;
            this.ClientSize = new System.Drawing.Size(669, 330);
            this.Controls.Add(this.MotorsOff);
            this.Controls.Add(this.MotorsOn);
            this.Controls.Add(this.toolStrip2);
            this.Controls.Add(this.groupBoxPosition);
            this.Controls.Add(this.btnClose);
            this.Controls.Add(this.tbOutput);
            this.Controls.Add(this.BtnCalibrate);
            this.Name = "Form1";
            this.Text = "KNI .NET demo";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBoxPosition.ResumeLayout(false);
            this.groupBoxPosition.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownPsi)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownTheta)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownPhi)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownZ)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownY)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownVel)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownX)).EndInit();
            this.toolStrip2.ResumeLayout(false);
            this.toolStrip2.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button BtnCalibrate;
        private System.Windows.Forms.OpenFileDialog openConfigFileDialog;
        private System.Windows.Forms.TextBox tbOutput;
        private System.Windows.Forms.Button btnClose;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.GroupBox groupBoxPosition;
        private System.Windows.Forms.Button buttonGo;
        private System.Windows.Forms.Button buttonRead;
        private System.Windows.Forms.NumericUpDown numericUpDownPsi;
        private System.Windows.Forms.NumericUpDown numericUpDownTheta;
        private System.Windows.Forms.NumericUpDown numericUpDownPhi;
        private System.Windows.Forms.NumericUpDown numericUpDownZ;
        private System.Windows.Forms.NumericUpDown numericUpDownY;
        private System.Windows.Forms.NumericUpDown numericUpDownX;
        private System.Windows.Forms.CheckBox checkBoxLM;
        private System.Windows.Forms.NumericUpDown numericUpDownVel;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.ToolStripButton toolStripConnect;
        private System.Windows.Forms.ToolStripSeparator toolStripSeparator4;
        private System.Windows.Forms.ToolStripLabel toolStripLabel1;
        private System.Windows.Forms.ToolStripTextBox toolStripIPAddress;
        private System.Windows.Forms.ToolStripSeparator toolStripSeparator3;
        private System.Windows.Forms.ToolStripLabel toolStripLabel2;
        private System.Windows.Forms.ToolStripTextBox toolStripConfigFile;
        private System.Windows.Forms.ToolStrip toolStrip2;
        private System.Windows.Forms.Button MotorsOn;
        private System.Windows.Forms.Button MotorsOff;
    }
}

