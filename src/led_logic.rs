use smart_leds::RGB8;

pub const HEIGHT: usize = 8;
pub const WIDTH: usize = 32;
const BRIGHTNESS: u8 = 4;

/// 4x4 bitmap font for digits 0-9
pub const DIGITS: &[&[u8; 4]] = &[
    &[0b0110, 0b1001, 0b1001, 0b0110], // 0
    &[0b0010, 0b1110, 0b0010, 0b1111], // 1
    &[0b1110, 0b0001, 0b0100, 0b1111], // 2
    &[0b0110, 0b1011, 0b0101, 0b0010], // 3
    &[0b0010, 0b0110, 0b1111, 0b0010], // 4
    &[0b0100, 0b1011, 0b1101, 0b0010], // 5
    &[0b0100, 0b1011, 0b1101, 0b1110], // 6
    &[0b1111, 0b0010, 0b0100, 0b1000], // 7
    &[0b1110, 0b1011, 0b1101, 0b0111], // 8
    &[0b1111, 0b1011, 0b1101, 0b0010], // 9
];

pub fn num_to_pixels(mut num: u64) -> [RGB8; WIDTH * HEIGHT] {
    let mut values = [RGB8::default(); WIDTH * HEIGHT];
    let mut index = 11;
    while num != 0 {
        let digit = (num % 10) as usize;
        num /= 10;

        let digit_bitmap = DIGITS[digit];

        for (row, &row_bits) in digit_bitmap.iter().enumerate() {
            for col in 0..=3 {
                if (row_bits >> (3 - col)) & 1 == 1 {
                    let x_padding = (index % (WIDTH / 5)) * 5 + 2;
                    let x = x_padding + col;
                    let y_shift = if index >= 6 { 4 } else { 0 };
                    let y = y_shift + row;
                    values[y * WIDTH + x] = RGB8 {
                        r: BRIGHTNESS,
                        g: 0,
                        b: 0,
                    };
                }
            }
        }

        if index == 0 {
            break;
        }

        index -= 1;
    }

    values
}

pub fn logical_array_to_zig_zag(arr: [RGB8; WIDTH * HEIGHT]) -> [RGB8; WIDTH * HEIGHT] {
    let mut output = [RGB8::default(); WIDTH * HEIGHT];

    for (pos, val) in arr.iter().enumerate() {
        let col = pos % WIDTH; // Column (strip number)
        let row = pos / WIDTH; // Row within that column

        let pixel_index = if col % 2 == 0 {
            // Even columns: normal order (top to bottom)
            col * HEIGHT + row
        } else {
            // Odd columns: reversed order (bottom to top)
            col * HEIGHT + (HEIGHT - 1 - row)
        }
        .min(WIDTH * HEIGHT - 1);

        output[pixel_index] = *val;
    }

    output
}
