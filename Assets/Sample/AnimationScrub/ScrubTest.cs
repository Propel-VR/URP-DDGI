using System.Collections.Generic;
using EasyButtons;
using UnityEngine;
using UnityEngine.UI;
using System.IO;



#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.Animations;
#endif

public class ScrubTest : MonoBehaviour
{
    [SerializeField] float scrubSpeed = 1f;
    [SerializeField] AnimationClip[] clips;
    [SerializeField] RectTransform markerTemplate;
    [SerializeField] Slider slider;
    [SerializeField] int recordFramerate = 60;

    [SerializeField] Animator animator;

    float scrubPosition;
    bool isPlaying;
    float totalLength;
    List<float> clipStart = new();
    List<int> clipHash = new();

    [Button]
    public void BuildController ()
    {
#if UNITY_EDITOR
        ScrubAnimatorBuilder.CreateControllerFromClips(clips, animator);
#endif
    }

    private void Awake()
    {
        // Compiling the clips, creating the markers
        var marker = Instantiate(markerTemplate);
        marker.SetParent(markerTemplate.parent);
        marker.gameObject.SetActive(true);
        marker.anchorMin = new Vector2(0f,0f);
        marker.anchorMax = new Vector2(0f,0f);
        marker.anchoredPosition = new Vector2(0f, 32f);
        foreach (var clip in clips)
        {
            clipStart.Add(totalLength);
            clipHash.Add(Animator.StringToHash(clip.name));
            totalLength += clip.length;
        }
        int i = 0;
        float currentTime = 0;
        foreach(var clip in clips)
        {
            currentTime += clip.length;

            marker = Instantiate(markerTemplate);
            marker.SetParent(markerTemplate.parent);
            marker.gameObject.SetActive(true);
            marker.anchorMin = new Vector2(currentTime / totalLength, 0f);
            marker.anchorMax = new Vector2(currentTime / totalLength, 0f);
            marker.anchoredPosition = new Vector2(0f, 32f);
            i++;
        }


        // Registering slider
        slider.maxValue = totalLength;
        slider.onValueChanged.AddListener((value) => OnSliderUpdated(value));
        
        
        animator.StopRecording();
        animator.speed = 0f;
    }

    private void Update()
    {
        if(Input.GetKeyUp(KeyCode.Space))
        {
            TogglePlay();
        }
        
        
        // Scrubbing
        if (isPlaying)
        {
            scrubPosition = Mathf.Clamp(scrubPosition + Time.deltaTime * scrubSpeed, 0, totalLength);
        }

        if (Input.GetKey(KeyCode.LeftArrow))
        {
            scrubPosition = Mathf.Clamp(scrubPosition - Time.deltaTime * scrubSpeed, 0, totalLength);

            isPlaying = false;
        }

        if (Input.GetKey(KeyCode.RightArrow))
        {
            scrubPosition = Mathf.Clamp(scrubPosition + Time.deltaTime * scrubSpeed, 0, totalLength);

            isPlaying = false;
        }

        // Playback
        int currentClip = 0;
        float normalizedPosition = 0f;
        for(int i = 0; i < clips.Length; i++)
        {
            if (scrubPosition > clipStart[i])
            {
                currentClip = i;
                normalizedPosition = scrubPosition - clipStart[i];
            }
        }
        normalizedPosition /= Mathf.Max(Mathf.Epsilon, clips[currentClip].length);
        animator.Play(clipHash[currentClip], 0, normalizedPosition);
        slider.SetValueWithoutNotify(scrubPosition);
    }

    private void OnSliderUpdated (float value)
    {
        scrubPosition = value;
    }

    

    public void TogglePlay ()
    {
        isPlaying = !isPlaying;
    }
}

#if UNITY_EDITOR

public static class ScrubAnimatorBuilder
{
    public static void CreateControllerFromClips(AnimationClip[] clips, Animator animator)
    {
        if (clips == null || clips.Length == 0) return;

        string filePath = AssetDatabase.GetAssetPath(clips[0]);
        string directory = Path.GetDirectoryName(filePath);
        string generatedPath = $"{directory}/{animator.gameObject.name}_GENERATED.controller";

        // Creates the controller
        var controller = AnimatorController.CreateAnimatorControllerAtPath(generatedPath);
        var rootStateMachine = controller.layers[0].stateMachine;

        var lastState = (AnimatorState)null;
        foreach(var clip in clips)
        {
            var state = rootStateMachine.AddState(clip.name);
            state.motion = clip;

            if (lastState != null)
            {
                var enterTransition = lastState.AddTransition(state);
                enterTransition.exitTime = 1f;
                enterTransition.duration = 0f;
                enterTransition.hasExitTime = true;

                var exitTransition = state.AddTransition(lastState);
                exitTransition.exitTime = 1f;
                exitTransition.duration = 0f;
                exitTransition.hasExitTime = false;
            }
            lastState = state;
        }


        AnimatorController.SetAnimatorController(animator, controller);
    }
}

#endif
