package com.mcm.nativetest;

import edu.wpi.first.wpilibj.system.NumericalIntegration;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.junit.Test;

import java.util.Arrays;

import static org.junit.Assert.assertEquals;

/**
 * Example local unit test, which will execute on the development machine (host).
 *
 * @see <a href="http://d.android.com/tools/testing">Testing documentation</a>
 */
public class ExampleUnitTest {
    @Test
    public void addition_isCorrect() {
        Matrix<N1, N1> y0 = VecBuilder.fill(0.0);

        //noinspection SuspiciousNameCombination
        Matrix<N1, N1> y1 =
                NumericalIntegration.rk4(
                        (Matrix<N1, N1> x) -> {
                            Matrix<N1, N1> y = new Matrix<>(Nat.N1(), Nat.N1());
                            y.set(0, 0, Math.exp(x.get(0, 0)));
                            return y;
                        },
                        y0,
                        0.1);

        System.out.println(Arrays.toString(y1.getData()));

        assertEquals(Math.exp(0.1) - Math.exp(0.0), y1.get(0, 0), 1e-3);
    }
}