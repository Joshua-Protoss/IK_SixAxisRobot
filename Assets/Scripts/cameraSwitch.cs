using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class cameraSwitch : MonoBehaviour
{
    public GameObject cameraFront;
    public GameObject cameraTop;
    public int Manager;

    public void ManagerCamera()
    {
        if (Manager == 0)
        {
            camFront();
            Manager = 1;
        }
        else
        {
            camTop();
            Manager = 0;
        }
    }
    void camFront()
    {
        cameraFront.SetActive(true);
        cameraTop.SetActive(false);
    }

    void camTop()
    {
        cameraFront.SetActive(false);
        cameraTop.SetActive(true);
    }
}
