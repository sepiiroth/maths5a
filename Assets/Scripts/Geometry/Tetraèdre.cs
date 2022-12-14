using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Tetraèdre
{
    private List<Triangle> faces;

    public Tetraèdre(Triangle t1, Triangle t2, Triangle t3, Triangle t4)
    {
        faces = new List<Triangle>();
        faces.Add(t1);
        faces.Add(t2);
        faces.Add(t3);
        faces.Add(t4);
    }

    public List<Triangle> GetFaces()
    {
        return faces;
    }

    public Vector3 GetSphereCenter()
    {
        List<Vector3> sommets = new List<Vector3>();
        faces.ForEach(x => sommets.AddRange(x.GetSommet()));
        sommets = sommets.Distinct().ToList();

        var a = sommets[1].x - sommets[0].x;
        var b = sommets[1].y - sommets[0].y;
        var c = sommets[1].z - sommets[0].z;
        
        var d = sommets[2].x - sommets[0].x;
        var e = sommets[2].y - sommets[0].y;
        var f = sommets[2].z - sommets[0].z;
        
        var g = sommets[3].x - sommets[0].x;
        var h = sommets[3].y - sommets[0].y;
        var i = sommets[3].z - sommets[0].z;

        var r1 = (Mathf.Pow(sommets[1].x, 2) + Mathf.Pow(sommets[1].y, 2) + Mathf.Pow(sommets[1].z, 2) -
                  Mathf.Pow(sommets[0].x, 2) - Mathf.Pow(sommets[0].y, 2) - Mathf.Pow(sommets[0].z, 2))/2;
        var r2 = (Mathf.Pow(sommets[2].x, 2) + Mathf.Pow(sommets[2].y, 2) + Mathf.Pow(sommets[2].z, 2) -
                  Mathf.Pow(sommets[0].x, 2) - Mathf.Pow(sommets[0].y, 2) - Mathf.Pow(sommets[0].z, 2))/2;
        var r3 = (Mathf.Pow(sommets[3].x, 2) + Mathf.Pow(sommets[3].y, 2) + Mathf.Pow(sommets[3].z, 2) -
                  Mathf.Pow(sommets[0].x, 2) - Mathf.Pow(sommets[0].y, 2) - Mathf.Pow(sommets[0].z, 2))/2;

        var detM = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);

        var mat = new float[3, 3]
        {
            {a, b, c},
            {d, e, f},
            {g, h, i}
        };


        var matT = new float[3, 3]
        {
            {a, d, g},
            {b, e, h},
            {c, f, i}
        };

        var am = (matT[1,1] * matT[2,2]) - (matT[1,2] * matT[2,1]);
        var bm = (matT[1,0] * matT[2,2]) - (matT[1,2] * matT[2,0]);
        var cm = (matT[1,0] * matT[2,1]) - (matT[1,1] * matT[2,0]);
        
        var dm = (matT[0,1] * matT[2,2]) - (matT[0,2] * matT[2,1]);
        var em = (matT[0,0] * matT[2,2]) - (matT[0,2] * matT[2,0]);
        var fm = (matT[0,0] * matT[2,1]) - (matT[0,1] * matT[2,0]);
        
        var gm = (matT[0,1] * matT[1,2]) - (matT[0,2] * matT[1,1]);
        var hm = (matT[0,0] * matT[1,2]) - (matT[0,2] * matT[1,0]);
        var im = (matT[0,0] * matT[1,1]) - (matT[0,1] * matT[1,0]);
        
        var adjM = new float[3, 3]
        {
            {am, -bm, cm},
            {-dm, em, -fm},
            {gm, -hm, im}
        };

        var oppDet = 1 / detM;
        

        var matInv = new float[3, 3]
        {
            {oppDet * adjM[0, 0], oppDet * adjM[0, 1], oppDet * adjM[0, 2]},
            {oppDet * adjM[1, 0], oppDet * adjM[1, 1], oppDet * adjM[1, 2]},
            {oppDet * adjM[2, 0], oppDet * adjM[2, 1], oppDet * adjM[2, 2]}
        };

        var xC = (matInv[0, 0] * r1) + (matInv[0, 1] * r2) + (matInv[0, 2] * r3);
        var yC = (matInv[1, 0] * r1) + (matInv[1, 1] * r2) + (matInv[1, 2] * r3);
        var zC = (matInv[2, 0] * r1) + (matInv[2, 1] * r2) + (matInv[2, 2] * r3);
        
        return new Vector3(xC, yC, zC);
    }
    
    public bool ContainsPoint(Vector3 P)
    {
        Vector3 c = GetSphereCenter();

        Vector3 s = faces[0].GetSommet()[0];

        float r = Mathf.Abs(Vector3.Distance(c, s));
        
        return Mathf.Abs(Vector3.Distance(c, P)) < r;
    }
}
