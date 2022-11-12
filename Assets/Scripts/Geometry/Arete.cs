using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Arete
{
    private Vector3 pointA, pointB;
    //private List<Triangle> trianglesAssocie;
    private LineRenderer lineRenderer;

    public Arete(Vector3 A, Vector3 B)
    {
        pointA = A;
        pointB = B;
    }

    public void SetLineRenderer(LineRenderer lr)
    {
        lineRenderer = lr;
    }

    public void SetPointA(Vector3 position)
    {
        pointA = position;
        lineRenderer.SetPosition(0, position);
    }
    
    public void SetPointB(Vector3 position)
    {
        pointB = position;
        lineRenderer.SetPosition(1, position);
    }

    public Vector3 GetPointA()
    {
        return pointA;
    }
    
    public Vector3 GetPointB()
    {
        return pointB;
    }

    public bool Equals(Arete A)
    {
        return (pointA.Equals(A.pointA) && pointB.Equals(A.pointB)) | (pointB.Equals(A.pointA) && pointA.Equals(A.pointB));
    }

    public String ToString()
    {
        return $"{pointA} - {pointB}";
    }

    /*public void AddTriangleAssocie(Triangle t)
    {
        trianglesAssocie.Add(t);
    }

    public List<Triangle> GetTrianglesAssocie()
    {
        return trianglesAssocie;
    }*/
}
