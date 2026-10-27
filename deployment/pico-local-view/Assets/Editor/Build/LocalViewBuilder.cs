using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>CI entry point for the side-by-side Local View application.</summary>
public static class LocalViewBuilder
{
    public static void BuildAndroid()
    {
        const string packageId = "com.swaaniida.xrobotoolkit.localview";
        const string productName = "XRoboToolkit Local View";

        string output = Path.GetFullPath(
            Path.Combine(Application.dataPath, "../Builds/Android/XRoboToolkit-Local-View.apk"));
        Directory.CreateDirectory(Path.GetDirectoryName(output));

        PlayerSettings.productName = productName;
        PlayerSettings.SetApplicationIdentifier(BuildTargetGroup.Android, packageId);
        PlayerSettings.bundleVersion = "1.1.1-local.7";
        PlayerSettings.Android.bundleVersionCode = 10007;
        PlayerSettings.Android.useCustomKeystore = false;
        EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Android, BuildTarget.Android);
        PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.Android, false);
        PlayerSettings.SetGraphicsAPIs(
            BuildTarget.Android,
            new[] { GraphicsDeviceType.OpenGLES3 });

        string[] scenes = EditorBuildSettings.scenes
            .Where(scene => scene.enabled)
            .Select(scene => scene.path)
            .ToArray();
        BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
        {
            scenes = scenes,
            locationPathName = output,
            target = BuildTarget.Android,
            options = BuildOptions.Development,
        });

        if (report.summary.result != BuildResult.Succeeded)
        {
            throw new Exception($"Local View APK build failed: {report.summary.result}");
        }
        Debug.Log($"LOCAL_VIEW_APK={output}");
    }
}
