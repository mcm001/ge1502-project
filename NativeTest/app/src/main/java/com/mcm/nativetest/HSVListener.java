package com.mcm.nativetest;

import android.widget.SeekBar;
import android.widget.TextView;
import org.photonvision.common.util.numbers.IntegerCouple;
import org.photonvision.vision.pipeline.ColoredShapePipelineSettings;

public class HSVListener implements SeekBar.OnSeekBarChangeListener {
    MainActivity mainActivity;
    TextView hueMinText, hueMaxText;
    SeekBar hueMin, hueMax;

    public HSVListener(MainActivity mainActivity) {
        hueMinText = mainActivity.findViewById(R.id.hueMinText);
        hueMin = mainActivity.findViewById(R.id.hueMin);
        hueMaxText = mainActivity.findViewById(R.id.hueMaxText);
        hueMax = mainActivity.findViewById(R.id.hueMax);
        hueMin.setOnSeekBarChangeListener(this);
        hueMax.setOnSeekBarChangeListener(this);
        this.mainActivity = mainActivity;
    }

    @Override
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
        VisionProcessThread thread = mainActivity.getVisionProcess();
        if(thread == null) return;
        if(!fromUser) return;

        ColoredShapePipelineSettings settings = thread.getColoredShapeSettings();
        settings.hsvHue = new IntegerCouple(hueMin.getProgress(), hueMax.getProgress());

        if(settings.hsvHue.getFirst() > settings.hsvHue.getSecond()) {
            int lower = settings.hsvHue.getSecond();
            int upper = settings.hsvHue.getFirst();
            settings.hsvHue = new IntegerCouple(lower, upper);
            hueMin.setProgress(lower);
            hueMax.setProgress(upper);
        }

        System.out.println(progress);

        hueMinText.setText("Hue Min: " + settings.hsvHue.getFirst());
        hueMaxText.setText("Hue Max: " + settings.hsvHue.getSecond());
        thread.setSettings(settings);
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {
    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {
    }
}
