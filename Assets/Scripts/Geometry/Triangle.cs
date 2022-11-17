using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Triangle
{
    private Arete arete1, arete2, arete3;
    public Point centreCirconscrit; 

    public Triangle(Arete A1, Arete A2, Arete A3)
    {
        arete1 = A1;
        arete2 = A2;
        arete3 = A3;
    }
    
    public Triangle(Vector3 V1, Vector3 V2, Vector3 V3)
    {
        List<Vector3> sommet = new List<Vector3>() {V1, V2, V3};
        sommet = MeshCreator.Instance().getConvexEnvelopJarvis(sommet);

        arete1 = new Arete(sommet[0], sommet[1]);
        arete2 = new Arete(sommet[1], sommet[2]);;
        arete3 = new Arete(sommet[2], sommet[0]);;
    }

    public Arete GetArete(int index)
    {
        switch (index)
        {
            case 1 :
                return arete1;
                break;
            case 2 :
                return arete1;
                break;
            case 3 :
                return arete1;
                break;
            default:
                return null;
        }
    }

    public List<Arete> GetAllArete()
    {
        return new List<Arete>() {arete1, arete2, arete3};
    }

    public List<Vector3> GetSommet()
    {
        List<Vector3> temp = new List<Vector3>();
        temp.Add(arete1.GetPointA());
        temp.Add(arete1.GetPointB());
        if (temp.Contains(arete2.GetPointA()))
        {
            temp.Add(arete2.GetPointB());
        }
        else
        {
            temp.Add(arete2.GetPointA());
        }

        return temp;
    }

    public bool ContainsArete(Arete A)
    {
        return A.Equals(arete1) | A.Equals(arete2) | A.Equals(arete3);
    }

    
    public String ToString()
    {
        return $"{arete1.ToString()} - {arete2.ToString()} - {arete3.ToString()}";
    }

    public bool ContainsPoint(Vector3 P)
    {
        List<Vector3> triangleSorted = MeshCreator.Instance().getConvexEnvelopJarvis(this.GetSommet());

        for (int i = 0; i < triangleSorted.Count-1; i++)
        {
            if (ProjectManager.Instance().isLeft(triangleSorted[i], triangleSorted[(i+1)], P))
            {
                return false;
            }
        }

        return true;
    }
}
