package com.state_machine.test;

import org.ejml.simple.SimpleMatrix;

public class testEJML {
    public static void main(String[] args){
        SimpleMatrix testM = new SimpleMatrix(3,2);
        double[][] a = new double[3][2];
        a[0][1] = 10;
        a[2][1] = 5;
        SimpleMatrix B = new SimpleMatrix(a);
        B = B.extractMatrix(0,SimpleMatrix.END,0,2);
        SimpleMatrix I = SimpleMatrix.identity(10);
        I.set(10);
        SimpleMatrix X = new SimpleMatrix(2,10);
        X.zero();

        SimpleMatrix M = new SimpleMatrix(5,5);
        M.set(1);
        SimpleMatrix N = new SimpleMatrix(5,5);
        N.set(2);
        SimpleMatrix R = new SimpleMatrix(10,5);
        R.insertIntoThis(0,0,M.combine(5,0,N));

        SimpleMatrix SZA = SimpleMatrix.identity(10).scale(10);

        System.out.print("test");
    }
}