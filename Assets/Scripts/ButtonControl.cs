using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Transformations;

public class ButtonControl : MonoBehaviour
{
    public Transform[] Robot = new Transform[6];
    public double increment = 100;
    private double jointMultiplier = 0.01;
    private bool jogPressed;
    private bool jogDirection; // true = counter clockwise
    private int jogAxis;

    // Joint Parameters
    private double[] joint = new double[6];
    private double[] TCP = new double[6];
    private double[] zero = new double[6];

 
    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < 6; i++)
        {
            joint[i] = 0.0;
        }

        Robot[0].localEulerAngles = new Vector3(0, (float)-joint[0], 0);
        Robot[1].localEulerAngles = new Vector3(0, 0, (float)-joint[1]);
        Robot[2].localEulerAngles = new Vector3(0, 0, (float)-joint[2]);
        Robot[3].localEulerAngles = new Vector3((float)-joint[3], 0, 0);
        Robot[4].localEulerAngles = new Vector3(0, 0, (float)-joint[4]);
        Robot[5].localEulerAngles = new Vector3((float)-joint[5], 0, 0);

        jogPressed = false;

        
        Arm.Direct(joint, zero, ref TCP);

       // Debug.Log("Initializing from ButtonControl in void start()");
    }

    // Update is called once per frame
    void Update()
    {
        if (jogDirection)
        {
            if (jogPressed)
            {
                TCP[jogAxis] += increment * jointMultiplier * Time.deltaTime;
                //joint[jogAxis] += increment * jointMultiplier; //forward kinematics
            }
        }
        else
        {
            if (jogPressed)
            {
                TCP[jogAxis] -= increment * jointMultiplier * Time.deltaTime;
                //joint[jogAxis] += increment * jointMultiplier; //forward kinematics
            }
        }

        Arm.Inverse(TCP, joint, ref joint);
        Robot[0].localEulerAngles = new Vector3(0, (float)-joint[0], 0);
        Robot[1].localEulerAngles = new Vector3(0, 0, (float)-joint[1]);
        Robot[2].localEulerAngles = new Vector3(0, 0, (float)-joint[2]);
        Robot[3].localEulerAngles = new Vector3((float)-joint[3], 0, 0);
        Robot[4].localEulerAngles = new Vector3(0, 0, (float)-joint[4]);
        Robot[5].localEulerAngles = new Vector3((float)-joint[5], 0, 0);
    }

    public void J1Plus()
    {
        jogDirection = true;
        jogPressed = !jogPressed;
        jogAxis = 0;
        jointMultiplier = 250;      //with time delta time
        //jointMultiplier = 0.8;
        //jointMultiplier = 0.03; // forward kinematics

    }
    public void J2Plus()
    {
        jogDirection = true;
        jogPressed = !jogPressed;
        jogAxis = 1;
        jointMultiplier = 250;
        //jointMultiplier = 0.8;
        //jointMultiplier = 0.03; // forward kinematics

    }
    public void J3Plus()
    {
        jogDirection = true;
        jogPressed = !jogPressed;
        jogAxis = 2;
        jointMultiplier = 250;
        //jointMultiplier = 0.8;
        //jointMultiplier = 0.03; // forward kinematics

    }
    public void J4Plus()
    {
        jogDirection = true;
        jogPressed = !jogPressed;
        jogAxis = 3;
        jointMultiplier = 0.07;

    }
    public void J5Plus()
    {
        jogDirection = true;
        jogPressed = !jogPressed;
        jogAxis = 4;
        jointMultiplier = 0.1;

    }
    public void J6Plus()
    {
        jogDirection = true;
        jogPressed = !jogPressed;
        jogAxis = 5;
        jointMultiplier = 0.2;

    }

    public void J1Minus()
    {
        jogDirection = false;
        jogPressed = !jogPressed;
        jogAxis = 0;
        //jointMultiplier = 0.03; // forward kinematics
        //jointMultiplier = 0.5;
        jointMultiplier = 250;

    }
    public void J2Minus()
    {
        jogDirection = false;
        jogPressed = !jogPressed;
        jogAxis = 1;
        //jointMultiplier = 0.03; forward
        //jointMultiplier = 0.5;
        jointMultiplier = 250;

    }
    public void J3Minus()
    {
        jogDirection = false;
        jogPressed = !jogPressed;
        jogAxis = 2;
        //jointMultiplier = 0.03; forward
        //jointMultiplier = 0.5;
        jointMultiplier = 250;

    }
    public void J4Minus()
    {
        jogDirection = false;
        jogPressed = !jogPressed;
        jogAxis = 3;
        jointMultiplier = 0.07;

    }
    public void J5Minus()
    {
        jogDirection = false;
        jogPressed = !jogPressed;
        jogAxis = 4;
        jointMultiplier = 0.1;

    }
    public void J6Minus()
    {
        jogDirection = false;
        jogPressed = !jogPressed;
        jogAxis = 5;
        jointMultiplier = 0.2;

    }


}
