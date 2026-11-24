/*
 * Simple Math Library
 * Version: 1.0.0
 */

const SimpleMath = {
    add: function (a, b) {
        return a + b;
    },

    subtract: function (a, b) {
        return a - b;
    },

    multiply: function (a, b) {
        return a * b;
    },

    divide: function (a, b) {
        if (b === 0) {
            throw new Error("Division by zero is not allowed");
        }
        return a / b;
    }
};

if (typeof module !== "undefined" && module.exports) {
    module.exports = SimpleMath;
}