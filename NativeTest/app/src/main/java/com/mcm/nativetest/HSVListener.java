package com.mcm.nativetest;

import android.widget.SeekBar;
import android.widget.TextView;
import org.photonvision.common.util.numbers.IntegerCouple;
import org.photonvision.vision.pipeline.ColoredShapePipelineSettings;

public class HSVListener implements SeekBar.OnSeekBarChangeListener {
    MainActivity mainActivity;
    TextView hueMinText, hueMaxText;
    SeekBar hueMin, hueMax;
    TextView satMinText, satMaxText;
    SeekBar satMin, satMax;
    TextView valueMinText, valueMaxText;
    SeekBar valueMin, valueMax;

    TextView accuracyText;
    SeekBar accuracy;

    public HSVListener(MainActivity mainActivity) {
        this.mainActivity = mainActivity;

        // Hue
        hueMinText = mainActivity.findViewById(R.id.hueMinText);
        hueMin = mainActivity.findViewById(R.id.hueMin);
        hueMaxText = mainActivity.findViewById(R.id.hueMaxText);
        hueMax = mainActivity.findViewById(R.id.hueMax);
        hueMin.setOnSeekBarChangeListener(this);
        hueMax.setOnSeekBarChangeListener(this);

        // Saturation
        satMinText = mainActivity.findViewById(R.id.satMinText);
        satMin = mainActivity.findViewById(R.id.satMin);
        satMaxText = mainActivity.findViewById(R.id.satMaxText);
        satMax = mainActivity.findViewById(R.id.satMax);
        satMin.setOnSeekBarChangeListener(this);
        satMax.setOnSeekBarChangeListener(this);

        // Value
        valueMinText = mainActivity.findViewById(R.id.valueMinText);
        valueMin = mainActivity.findViewById(R.id.valueMin);
        valueMaxText = mainActivity.findViewById(R.id.valueMaxText);
        valueMax = mainActivity.findViewById(R.id.valueMax);
        valueMin.setOnSeekBarChangeListener(this);
        valueMax.setOnSeekBarChangeListener(this);

        // Accuracy percentage

    }

    @Override
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
        VisionProcessThread thread = mainActivity.getVisionProcess();
        if(thread == null) return;
        if(!fromUser) return;

        ColoredShapePipelineSettings settings = thread.getColoredShapeSettings();
        settings.hsvHue = new IntegerCouple(hueMin.getProgress(), hueMax.getProgress());
        settings.hsvSaturation = new IntegerCouple(satMin.getProgress(), satMax.getProgress());
        settings.hsvValue = new IntegerCouple(valueMin.getProgress(), valueMax.getProgress());

        // hue
        if(settings.hsvHue.getFirst() > settings.hsvHue.getSecond()) {
            int lower = settings.hsvHue.getSecond();
            int upper = settings.hsvHue.getFirst();
            settings.hsvHue = new IntegerCouple(lower, upper);
            hueMin.setProgress(lower);
            hueMax.setProgress(upper);
        }

        // sat
        if(settings.hsvSaturation.getFirst() > settings.hsvSaturation.getSecond()) {
            int lower = settings.hsvSaturation.getSecond();
            int upper = settings.hsvSaturation.getFirst();
            settings.hsvSaturation = new IntegerCouple(lower, upper);
            satMin.setProgress(lower);
            satMax.setProgress(upper);
        }

        // value
        if(settings.hsvValue.getFirst() > settings.hsvValue.getSecond()) {
            int lower = settings.hsvValue.getSecond();
            int upper = settings.hsvValue.getFirst();
            settings.hsvValue = new IntegerCouple(lower, upper);
            valueMin.setProgress(lower);
            valueMax.setProgress(upper);
        }

        hueMinText.setText("Hue Min: " + settings.hsvHue.getFirst());
        hueMaxText.setText("Hue Max: " + settings.hsvHue.getSecond());
        satMinText.setText("Sat Min: " + settings.hsvSaturation.getFirst());
        satMaxText.setText("Sat Max: " + settings.hsvSaturation.getSecond());
        valueMinText.setText("Value Min: " + settings.hsvValue.getFirst());
        valueMaxText.setText("Value Max: " + settings.hsvValue.getSecond());
        thread.setSettings(settings);
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {
    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {
    }
}
