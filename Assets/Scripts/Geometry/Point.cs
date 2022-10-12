using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Point : MonoBehaviour
{
    private Vector3 position;
    
    [SerializeField]
    private GameObject prefab;

    public Point(float x, float y, float z)
    {
        this.position = new Vector3(x, y, z);
    }
}
