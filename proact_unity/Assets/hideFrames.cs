using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// disable this if calibrating hololens for mocap 

public class hideFrames : MonoBehaviour
{
    public Transform holoworldParent;
    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < holoworldParent.childCount; i++)
            holoworldParent.GetChild(i).GetComponent<Renderer>().enabled = false;

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
