use std::sync::Arc;

/// Generic 2D image matrix with arbitrary element type.
/// Data is stored row-major with interleaved channels:
/// element at (row r, col c, channel ch) = data[r * cols * channels + c * channels + ch]
#[derive(Clone)]
pub struct Mat<T: Clone> {
    rows: usize,
    cols: usize,
    channels: usize,
    data: Arc<Vec<T>>,
    // offset into data (for sharing sub-views; usually 0)
    offset: usize,
}

impl<T: Clone + Default> Mat<T> {
    pub fn new(rows: usize, cols: usize, channels: usize) -> Self {
        Mat {
            rows,
            cols,
            channels,
            data: Arc::new(vec![T::default(); rows * cols * channels]),
            offset: 0,
        }
    }
}

impl<T: Clone> Mat<T> {
    pub fn from_data(rows: usize, cols: usize, channels: usize, data: Vec<T>) -> Self {
        assert_eq!(data.len(), rows * cols * channels);
        Mat {
            rows,
            cols,
            channels,
            data: Arc::new(data),
            offset: 0,
        }
    }

    pub fn rows(&self) -> usize {
        self.rows
    }
    pub fn cols(&self) -> usize {
        self.cols
    }
    pub fn height(&self) -> usize {
        self.rows
    }
    pub fn width(&self) -> usize {
        self.cols
    }
    pub fn channels(&self) -> usize {
        self.channels
    }
    pub fn pixels(&self) -> usize {
        self.rows * self.cols
    }

    #[inline]
    fn idx(&self, r: usize, c: usize, ch: usize) -> usize {
        self.offset + r * self.cols * self.channels + c * self.channels + ch
    }

    pub fn at(&self, r: usize, c: usize, ch: usize) -> &T {
        &self.data[self.idx(r, c, ch)]
    }

    pub fn at_mut(&mut self, r: usize, c: usize, ch: usize) -> &mut T {
        let idx = self.idx(r, c, ch);
        &mut Arc::make_mut(&mut self.data)[idx]
    }

    // For single-channel matrices
    pub fn at2(&self, r: usize, c: usize) -> &T {
        self.at(r, c, 0)
    }

    pub fn at2_mut(&mut self, r: usize, c: usize) -> &mut T {
        self.at_mut(r, c, 0)
    }

    pub fn set(&mut self, r: usize, c: usize, ch: usize, val: T) {
        let idx = self.idx(r, c, ch);
        Arc::make_mut(&mut self.data)[idx] = val;
    }

    pub fn set2(&mut self, r: usize, c: usize, val: T) {
        self.set(r, c, 0, val);
    }

    /// Pointer (slice) to start of row r
    pub fn row(&self, r: usize) -> &[T] {
        let start = self.offset + r * self.cols * self.channels;
        &self.data[start..start + self.cols * self.channels]
    }

    pub fn row_mut(&mut self, r: usize) -> &mut [T] {
        let start = self.offset + r * self.cols * self.channels;
        let end = start + self.cols * self.channels;
        &mut Arc::make_mut(&mut self.data)[start..end]
    }

    /// Pointer (slice) to pixel (r, c)
    pub fn pixel(&self, r: usize, c: usize) -> &[T] {
        let start = self.idx(r, c, 0);
        &self.data[start..start + self.channels]
    }

    pub fn pixel_mut(&mut self, r: usize, c: usize) -> &mut [T] {
        let start = self.idx(r, c, 0);
        let end = start + self.channels;
        &mut Arc::make_mut(&mut self.data)[start..end]
    }

    pub fn data(&self) -> &[T] {
        &self.data[self.offset..]
    }

    pub fn data_mut(&mut self) -> &mut [T] {
        let offset = self.offset;
        &mut Arc::make_mut(&mut self.data)[offset..]
    }

    pub fn clone_data(&self) -> Mat<T> {
        let v: Vec<T> = self.data().to_vec();
        Mat::from_data(self.rows, self.cols, self.channels, v)
    }
}

pub type Mat32f = Mat<f32>;
pub type Matuc = Mat<u8>;
