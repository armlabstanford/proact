using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WorkspaceHighlighter2 : MonoBehaviour
{
    public Transform armbaseTransform;
    public Transform tableTransform;
    public Transform publishedTransform;

    // Start is called before the first frame update
    void Start()
    {
        publishedTransform.rotation = new Quaternion(0, 0, 0, 1);
    }

    // Update is called once per frame
    void Update()
    {
        publishedTransform.position = new Vector3(armbaseTransform.position.x, tableTransform.position.y + (float)0.03, armbaseTransform.position.z + (float)0.5);
    }
}
