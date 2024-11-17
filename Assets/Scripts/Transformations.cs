using System;

namespace Transformations
{
    public class Arm
    {
        private static double TRF_EPSILON = 1E-05;
        private static double sind(double x) => Math.Sin(x * Math.PI / 180.0);  // input in degree -> output sin
        private static double cosd(double x) => Math.Cos(x * Math.PI / 180.0); // input in degree -> output cos
        private static double tand(double x) => Math.Tan(x * Math.PI / 180.0); // input in degree -> output tan
        private static double atan2d(double y, double x) => Math.Atan2(y, x) * 180.0 / Math.PI; // calculate tan 

        public static void testingDebug()
        {
            UnityEngine.Debug.Log("This is come from Transformations");
        }
        
        private static double Modulo2PI(double actual, double desired)
        {
            double num1 = actual - desired;
            int num2 = ((int)num1 + 180 * Math.Sign(num1)) / 360;
            return actual - 360.0 * (double)num2;
        }

        private static short MatMult(double[,] M1, double[,] M2, double[,] M3)
        {
            for (int index1 = 0; index1 < 3; ++index1)
            {
                for (int index2 = 0; index2 < 3; ++index2)
                    M3[index1, index2] = 0.0;
            }
            for (int index3 = 0; index3 < 3; ++index3)
            {
                for (int index4 = 0; index4 < 3; ++index4)
                {
                    for (int index5 = 0; index5 < 3; ++index5)
                        M3[index3, index4] = M3[index3, index4] + M1[index3, index5] * M2[index5, index4];
                }
            }
            return 0;
        }

        private static int DecomposeMatrix(
            double[,] RM,
            double A_actual,
            double B_actual,
            double C_actual,
            ref double A,
            ref double B,
            ref double C)
        {
            double[] numArray1 = new double[2];
            double[] numArray2 = new double[2];
            double[] numArray3 = new double[2];
            double[] numArray4 = new double[2];
            double x = Math.Sqrt(1.0 - RM[2,0] * RM[2,0]);
            numArray2[0] = Arm.atan2d(-RM[2, 0], x);
            numArray2[1] = Arm.atan2d(-RM[2, 0], -x);
            if (Math.Abs(x) > Arm.TRF_EPSILON)
            {
                numArray3[0] = Arm.atan2d(RM[1, 0], RM[0, 0]);
                numArray3[1] = Arm.atan2d(-RM[1,0], -RM[0,0]);
                numArray1[0] = Arm.atan2d(RM[2, 1], RM[2, 2]);
                numArray1[1] = Arm.atan2d(-RM[2,1], -RM[2,2]);
            }
            else
            {
                numArray1[0] = numArray1[1] = A_actual;
                numArray3[0] = numArray3[1] = A_actual - (double) Math.Sign(-RM[2,0]) * Arm.atan2d(RM[0,1] * (double) Math.Sign(-RM[2,0]), RM[1,1]);
            }
            numArray1[0] = Arm.Modulo2PI(numArray1[0], A_actual);
            numArray1[1] = Arm.Modulo2PI(numArray1[1], A_actual);
            numArray3[0] = Arm.Modulo2PI(numArray3[0], C_actual);
            numArray3[1] = Arm.Modulo2PI(numArray3[1], C_actual);
            numArray4[0] = Math.Abs(numArray1[0] - A_actual) + Math.Abs(numArray2[0] - B_actual) + Math.Abs(numArray3[0] - C_actual);
            numArray4[1] = Math.Abs(numArray1[1] - A_actual) + Math.Abs(numArray2[1] - B_actual) + Math.Abs(numArray3[1] - C_actual);
            if (B_actual < 0.0)
            {
                if (numArray2[0] < 0.0 && numArray2[1] >= 0.0)
                {
                    A = numArray1[0];
                    B = numArray2[0];
                    C = numArray3[0];
                }
                else if (numArray2[1] < 0.0 && numArray2[0] >= 0.0)
                {
                    A = numArray1[1];
                    B = numArray2[1];
                    C = numArray3[1];
                }
                else if (numArray4[0] <= numArray4[1])
                {
                    A = numArray1[0];
                    B = numArray2[0];
                    C = numArray3[0];
                }
                else
                {
                    A = numArray1[1];
                    B = numArray2[1];
                    C = numArray3[1];
                }
            }
            else if (numArray2[0] >= 0.0 && numArray2[1] < 0.0)
            {
                A = numArray1[0];
                B = numArray2[0];
                C = numArray3[0];
            }
            else if (numArray2[1] >= 0.0 && numArray2[0] < 0.0)
            {
                A = numArray1[1];
                B = numArray2[1];
                C = numArray3[1];
            }
            else if (numArray4[0] <= numArray4[1])
            {
                A = numArray1[0];
                B = numArray2[0];
                C = numArray3[0];
            }
            else
            {
                A = numArray1[1];
                B = numArray2[1];
                C = numArray3[1];
            }
            C = Arm.Modulo2PI(C, C_actual);
            return 0;
        }

        private static int ComposeMatrix(double[,] RM, double A, double B, double C)
        {
            RM[0, 0] = Arm.cosd(C) * Arm.cosd(B);
            RM[0, 1] = Arm.cosd(C) * Arm.sind(B) * Arm.sind(A) - Arm.sind(C) * Arm.cosd(A);
            RM[0, 2] = Arm.cosd(C) * Arm.sind(B) * Arm.cosd(A) + Arm.sind(C) * Arm.sind(A);
            RM[1, 0] = Arm.sind(C) * Arm.cosd(B);
            RM[1, 1] = Arm.sind(C) * Arm.sind(B) * Arm.sind(A) + Arm.cosd(C) * Arm.cosd(A);
            RM[1, 2] = Arm.sind(C) * Arm.sind(B) * Arm.cosd(A) - Arm.cosd(C) * Arm.sind(A);
            RM[2, 0] = -Arm.sind(B);
            RM[2, 1] = Arm.cosd(B) * Arm.sind(A);
            RM[2, 2] = Arm.cosd(B) * Arm.cosd(A);
            return 0;
        }

        public static int Direct(double[] JointAxes, double[] PathAxes, ref double[] Axes)
        {
            double jointAx1 = JointAxes[0];
            double jointAx2 = JointAxes[1];
            double jointAx3 = JointAxes[2];
            double jointAx4 = JointAxes[3];
            double jointAx5 = JointAxes[4];
            double jointAx6 = JointAxes[5];
            double num1 = 400.0;
            double num2 = 680.0;
            double num3 = 1100.0;
            double num4 = 766.0;
            double num5 = 230.0;
            double num6 = 345.0;
            double num7 = 244.0;
            double[] numArray = new double[6];
            double num8 = Arm.cosd(jointAx3) * (num4 + num6 + Arm.cosd(jointAx5) * num7) + Arm.sind(jointAx3) * (num5 - Arm.cosd(jointAx4) * Arm.sind(jointAx5) * num7);
            double num9 = -Arm.sind(jointAx3) * (num4 + num6 + Arm.cosd(jointAx5) * num7) + Arm.cosd(jointAx3) * (num5 - Arm.cosd(jointAx4) * Arm.sind(jointAx5) * num7);
            double num10 = num1 + Arm.cosd(jointAx2) * num8 + Arm.sind(jointAx2) * (num9 + num3);
            double num11 = num2 - Arm.sind(jointAx2) * num8 + Arm.cosd(jointAx2) * (num9 + num3);
            numArray[0] = Arm.cosd(jointAx1) * num10 - Arm.sind(jointAx1) * Arm.sind(jointAx4) * Arm.sind(jointAx5) * num7;
            numArray[1] = Arm.sind(jointAx1) * num10 + Arm.cosd(jointAx1) * Arm.sind(jointAx4) * Arm.sind(jointAx5) * num7;
            numArray[2] = num11;
            double[,] RM = new double[3, 3];
            double num12 = -Arm.sind(jointAx1) * Arm.cosd(jointAx4) + Arm.cosd(jointAx1) * Arm.sind(jointAx2 + jointAx3) * Arm.sind(jointAx4);
            double num13 = Arm.cosd(jointAx1) * Arm.cosd(jointAx2 + jointAx3) * Arm.sind(jointAx5) + Arm.cosd(jointAx5) * Arm.sind(jointAx1) * Arm.sind(jointAx4) + Arm.cosd(jointAx5) * Arm.cosd(jointAx1) * Arm.sind(jointAx2 + jointAx3) * Arm.cosd(jointAx4);
            double num14 = Arm.cosd(jointAx1) * Arm.cosd(jointAx4) + Arm.sind(jointAx1) * Arm.sind(jointAx2 + jointAx3) * Arm.sind(jointAx4);
            double num15 = Arm.sind(jointAx1) * Arm.cosd(jointAx2 + jointAx3) * Arm.sind(jointAx5) - Arm.cosd(jointAx1) * Arm.sind(jointAx4) * Arm.cosd(jointAx5) + Arm.cosd(jointAx5) * Arm.sind(jointAx1) * Arm.sind(jointAx2 + jointAx3) * Arm.cosd(jointAx4);
            double num16 = Arm.cosd(jointAx2 + jointAx3) * Arm.sind(jointAx4);
            double num17 = -Arm.sind(jointAx2 + jointAx3) * Arm.sind(jointAx5) + Arm.cosd(jointAx2 + jointAx3) * Arm.cosd(jointAx4) * Arm.cosd(jointAx5);
            RM[0, 0] = Arm.cosd(jointAx1) * Arm.cosd(jointAx2 + jointAx3) * Arm.cosd(jointAx5) - Arm.sind(jointAx1) * Arm.sind(jointAx4) * Arm.sind(jointAx5) - Arm.cosd(jointAx1) * Arm.sind(jointAx2 + jointAx3) * Arm.cosd(jointAx4) * Arm.sind(jointAx5);
            RM[0, 1] = Arm.cosd(jointAx6) * num12 + Arm.sind(jointAx6) * num13;
            RM[0, 2] = -Arm.sind(jointAx6) * num12 + Arm.cosd(jointAx6) * num13;
            RM[1, 0] = Arm.sind(jointAx1) * Arm.cosd(jointAx2 + jointAx3) * Arm.cosd(jointAx5) + Arm.cosd(jointAx1) * Arm.sind(jointAx4) * Arm.sind(jointAx5) - Arm.sind(jointAx1) * Arm.sind(jointAx2 + jointAx3) * Arm.cosd(jointAx4) * Arm.sind(jointAx5);
            RM[1, 1] = Arm.cosd(jointAx6) * num14 + Arm.sind(jointAx6) * num15;
            RM[1, 2] = -Arm.sind(jointAx6) * num14 + Arm.cosd(jointAx6) * num15;
            RM[2, 0] = -Arm.sind(jointAx2 + jointAx3) * Arm.cosd(jointAx5) - Arm.cosd(jointAx2 + jointAx3) * Arm.cosd(jointAx4) * Arm.sind(jointAx5);
            RM[2, 1] = Arm.cosd(jointAx6) * num16 + Arm.sind(jointAx6) * num17;
            RM[2, 2] = -Arm.sind(jointAx6) * num16 + Arm.cosd(jointAx6) * num17;
            double A = 0.0;
            double B = 0.0;
            double C = 0.0;
            Arm.DecomposeMatrix(RM, PathAxes[3], PathAxes[4], PathAxes[5], ref A, ref B, ref C);
            numArray[3] = A;
            numArray[4] = B;
            numArray[5] = C;
            for (int index = 0; index < 6; ++index)
                Axes[index] = numArray[index];

            return 0;
        }

        public static int Inverse(double[] PathAxes, double[] JointAxes, ref double[] Axes)
        {
            ushort num1 = 8;
            ushort num2 = 2;
            ushort num3 = 0;
            ushort num4 = 0;
            int maxValue = (int)byte.MaxValue;
            double[] numArray1 = new double[6];
            double pathAx1 = PathAxes[0];
            double pathAx2 = PathAxes[1]; 
            double pathAx3 = PathAxes[2];   
            double pathAx4 = PathAxes[3];
            double pathAx5 = PathAxes[4];
            double pathAx6 = PathAxes[5];
            double num5 = 400.0;
            double num6 = 680.0;
            double num7 = 1100.0;
            double num8 = 766.0;
            double x1 = 230.0;
            double num9 = 345.0;
            double num10 = 244.0;
            ushort num11 = 0;
            ushort num12 = JointAxes[2] >= -90.0 ? (ushort) ((uint) num11 | (uint) num3) : (ushort) ((uint) num11 | (uint) num2);
            ushort num13 = JointAxes[4] >= 0.0 ? (ushort)((uint)num12 | (uint)num4) : (ushort)((uint)num12 | (uint)num1);
            double[,] numArray2 = new double[3, 3];
            Arm.ComposeMatrix(numArray2, pathAx4, pathAx5, pathAx6);
            numArray1[0] = pathAx1 - num10 * numArray2[0, 0];
            numArray1[1] = pathAx2 - num10 * numArray2[1, 0];
            numArray1[2] = pathAx3 - num10 * numArray2[2, 0];
            int num14 = Math.Abs(numArray1[0]) >= Arm.TRF_EPSILON ? 1 : (Math.Abs(numArray1[1]) >= Arm.TRF_EPSILON ? 1 : 0);
            Axes[0] = num14 != 0 ? Arm.atan2d(numArray1[1], numArray1[0]) : JointAxes[0];
            Axes[0] = Arm.Modulo2PI(Axes[0], JointAxes[0]);
            double num15 = Math.Sqrt((num8 + num9) * (num8 + num9) + x1 * x1);
            double y = numArray1[2] - num6;
            double num16 = Math.Sqrt(numArray1[0] * numArray1[0] + numArray1[1] * numArray1[1]);
            if (Math.Abs(Arm.Modulo2PI(Axes[0] - Arm.atan2d(numArray1[1], numArray1[0]), 0.0)) > 90.0)
                num16 = -num16;
            double x2 = num16 - num5;
            double num17 = Math.Sqrt(x2 * x2 + y * y);
            if (num17 > num7 + num15 + Arm.TRF_EPSILON || num17 < Math.Abs(num7 - num15) - Arm.TRF_EPSILON)
                return maxValue;
            if (num17 > num7 + num15)
                num17 = num7 + num15;
            else if (num17 < Math.Abs(num7 = num15))
                num17 = Math.Abs(num7 - num15);
            double num18 = Arm.atan2d(y, x2);
            double x3 = (num17 * num17 + num7 * num7 - num15 * num15) / (2.0 * num7 * num17);
            double num19 = Arm.atan2d(Math.Sqrt(1.0 - x3 * x3), x3);
            double x4 = (num7 * num7 + num15 * num15 - num17 * num17) / (2.0 * num7 * num15);
            double num20 = 180.0 - Arm.atan2d(Math.Sqrt(1.0 - x4 * x4), x4);
            if (((int) num13 & (int) num2) != 0)
            {
                Axes[1] = 90.0 - num18 + num19;
                Axes[2] = -num20 - Arm.atan2d(num8 + num9, x1);
            }
            else
            {
                Axes[1] = 90.0 - num18 - num19;
                Axes[2] = num20 - Arm.atan2d(num8 + num9, x1);
            }
            double[,] M1 = new double[3, 3];
            double[,] M3 = new double[3, 3];
            double x5 = Axes[1] + Axes[2];
            double x6 = Axes[0];
            M1[0, 0] = Arm.cosd(x6) * Arm.cosd(x5);
            M1[0, 1] = -Arm.sind(x6);
            M1[0, 2] = Arm.sind(x5) * Arm.cosd(x6);
            M1[1, 0] = Arm.cosd(x5) * Arm.sind(x6);
            M1[1, 1] = Arm.cosd(x6);
            M1[1, 2] = Arm.sind(x5) * Arm.sind(x6);
            M1[2, 0] = -Arm.sind(x5);
            M1[2, 1] = 0.0;
            M1[2, 2] = Arm.cosd(x5);
            double num21 = M1[0, 1];
            M1[0, 1] = M1[1, 0];
            M1[1, 0] = num21;
            double num22 = M1[0, 2];
            M1[0, 2] = M1[2, 0];
            M1[2, 0] = num22;
            double num23 = M1[1, 2];
            M1[1, 2] = M1[2, 1];
            M1[2, 1] = num23;
            int num24 = (int)Arm.MatMult(M1, numArray2, M3);
            double[] numArray3 = new double[2];
            double[] numArray4 = new double[2];
            double[] numArray5 = new double[2];
            double[] numArray6 = new double[2];
            double jointAx1 = JointAxes[3];
            double jointAx2 = JointAxes[4];
            double jointAx3 = JointAxes[5];
            numArray4[0] = Arm.atan2d(Math.Sqrt(1.0 - M3[0, 0] * M3[0, 0]), M3[0, 0]);
            numArray4[1] = Arm.atan2d(-Math.Sqrt(1.0 - M3[0, 0] * M3[0, 0]), M3[0,0]);
            if (Math.Abs(numArray4[0]) > Arm.TRF_EPSILON)
            {
                numArray5[0] = Arm.atan2d(M3[0, 1], M3[0,2]);
                numArray5[1] = Arm.atan2d(-M3[0,1], -M3[0,2]);
                numArray3[0] = Arm.atan2d(M3[1,0], -M3[2,0]);
                numArray3[1] = Arm.atan2d(-M3[1,0], M3[2,0]);
            }
            else
            {
                numArray3[0] = numArray3[1] = jointAx1;
                numArray5[0] = numArray5[1] = Arm.atan2d(-M3[1,2], M3[2,2]) - jointAx1;
            }
            numArray3[0] = Arm.Modulo2PI(numArray3[0], JointAxes[3]);
            numArray3[1] = Arm.Modulo2PI(numArray3[1], JointAxes[3]);
            numArray5[0] = Arm.Modulo2PI(numArray5[0], JointAxes[5]);
            numArray5[1] = Arm.Modulo2PI(numArray5[1], JointAxes[5]);
            numArray6[0] = Math.Abs(numArray3[0] - jointAx1) + Math.Abs(numArray4[0] - jointAx2) + Math.Abs(numArray5[0] - jointAx3);
            numArray6[1] = Math.Abs(numArray3[1] - jointAx1) + Math.Abs(numArray4[1] - jointAx2) + Math.Abs(numArray5[1] - jointAx3);
            if (((int) num13 & (int) num1 ) != 0)
            {
                if (numArray4[0] < 0.0 && numArray4[1] >= 0.0)
                {
                    Axes[3] = numArray3[0];
                    Axes[4] = numArray4[0];
                    Axes[5] = numArray5[0];
                }
                else if (numArray4[1] < 0.0 && numArray4[0] >= 0.0)
                {
                    Axes[3] = numArray3[1];
                    Axes[4] = numArray4[1];
                    Axes[5] = numArray5[1];
                }
                else if (numArray6[0] <= numArray6[1])
                {
                    Axes[3] = numArray3[0];
                    Axes[4] = numArray4[0];
                    Axes[5] = numArray5[0];
                }
                else
                {
                    Axes[3] = numArray3[1];
                    Axes[4] = numArray4[1];
                    Axes[5] = numArray5[1];
                }
            }
            else if (numArray4[0] >= 0.0 && numArray4[1] < 0.0)
            {
                Axes[3] = numArray3[0];
                Axes[4] = numArray4[0];
                Axes[5] = numArray5[0];
            }
            else if (numArray4[1] >= 0.0 && numArray4[0] < 0.0)
            {
                Axes[3] = numArray3[1];
                Axes[4] = numArray4[1];
                Axes[5] = numArray5[1];
            }
            else if (numArray6[0] <= numArray6[1])
            {
                Axes[3] = numArray3[0];
                Axes[4] = numArray4[0];
                Axes[5] = numArray5[0];
            }
            else
            {
                Axes[3] = numArray3[1];
                Axes[4] = numArray4[1];
                Axes[5] = numArray5[1];
            }
            Axes[3] = Arm.Modulo2PI(Axes[3], JointAxes[3]);
            Axes[5] = Arm.Modulo2PI(Axes[5], JointAxes[5]);
            return 0;
        }

    }
}
