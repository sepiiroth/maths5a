using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Triangle
{
    private Arete arete1, arete2, arete3;

    public Triangle(Arete A1, Arete A2, Arete A3)
    {
        arete1 = A1;
        arete2 = A2;
        arete3 = A3;
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
}
