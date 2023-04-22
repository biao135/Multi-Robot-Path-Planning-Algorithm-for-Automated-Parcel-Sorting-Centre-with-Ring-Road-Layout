using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Threading;

public class Timer : MonoBehaviour
{
    private float deltaTime;
    private float timer;
    private TextMeshPro text;
    [SerializeField]
    private float stopTime;

    // Start is called before the first frame update
    void Start()
    {
        deltaTime = 0.02f;
        timer = 0;
        text = gameObject.GetComponent<TMPro.TextMeshPro>();
    }

    // Update is called once per frame
    void Update()
    {
        timer += deltaTime;
        int time = ((int)timer);
        text.text = time.ToString();
        if(time >= stopTime){
            Debug.Break();
            return;
        }
    }
}
