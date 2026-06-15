use ffmpeg_next as ffmpeg;
use ffmpeg::format::Pixel;
use ffmpeg::software::scaling::{context::Context as SwsCtx, flag::Flags};

pub struct VideoReader {
    ictx: ffmpeg::format::context::Input,
    decoder: ffmpeg::decoder::Video,
    stream_idx: usize,
    fmt_conv: Option<SwsCtx>,
    yuv_tmp: ffmpeg::frame::Video,
    pub width: u32,
    pub height: u32,
    pub fps: ffmpeg::Rational,
    pub total_frames: i64,
}

impl VideoReader {
    pub fn open(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let ictx = ffmpeg::format::input(path)?;
        let stream = ictx
            .streams()
            .best(ffmpeg::media::Type::Video)
            .ok_or("no video stream")?;
        let stream_idx = stream.index();
        let fps = stream.avg_frame_rate();
        let total_frames = stream.frames();

        let dec_ctx = ffmpeg::codec::context::Context::from_parameters(stream.parameters())?;
        let decoder = dec_ctx.decoder().video()?;
        let src_fmt = decoder.format();
        let width = decoder.width();
        let height = decoder.height();

        drop(stream);

        let fmt_conv = if src_fmt != Pixel::YUV420P {
            Some(SwsCtx::get(
                src_fmt, width, height,
                Pixel::YUV420P, width, height,
                Flags::BILINEAR,
            )?)
        } else {
            None
        };
        let yuv_tmp = ffmpeg::frame::Video::new(Pixel::YUV420P, width, height);

        Ok(VideoReader {
            ictx,
            decoder,
            stream_idx,
            fmt_conv,
            yuv_tmp,
            width,
            height,
            fps,
            total_frames,
        })
    }

    /// Returns the next decoded frame as YUV420P, or None at end of stream.
    pub fn next_frame(&mut self) -> Result<Option<ffmpeg::frame::Video>, ffmpeg::Error> {
        let mut decoded = ffmpeg::frame::Video::empty();
        // Drain any already-buffered frames first.
        if self.decoder.receive_frame(&mut decoded).is_ok() {
            return Ok(Some(self.to_yuv(decoded)?));
        }
        loop {
            // Feed packets until we get a frame.
            let mut got = false;
            let mut pkt_stream_idx = 0usize;
            let mut pkt_data: Option<ffmpeg::Packet> = None;

            for (stream, packet) in self.ictx.packets() {
                if stream.index() == self.stream_idx {
                    pkt_stream_idx = stream.index();
                    pkt_data = Some(packet);
                    got = true;
                    break;
                }
            }

            if !got {
                // EOF — flush decoder.
                self.decoder.send_eof()?;
                if self.decoder.receive_frame(&mut decoded).is_ok() {
                    return Ok(Some(self.to_yuv(decoded)?));
                }
                return Ok(None);
            }

            let _ = pkt_stream_idx;
            self.decoder.send_packet(pkt_data.as_ref().unwrap())?;
            if self.decoder.receive_frame(&mut decoded).is_ok() {
                return Ok(Some(self.to_yuv(decoded)?));
            }
        }
    }

    fn to_yuv(&mut self, frame: ffmpeg::frame::Video) -> Result<ffmpeg::frame::Video, ffmpeg::Error> {
        if let Some(ref mut conv) = self.fmt_conv {
            conv.run(&frame, &mut self.yuv_tmp)?;
            // Clone the data into a new frame so yuv_tmp stays reusable.
            let mut out = ffmpeg::frame::Video::new(Pixel::YUV420P, self.width, self.height);
            copy_yuv(&self.yuv_tmp, &mut out);
            Ok(out)
        } else {
            Ok(frame)
        }
    }

    pub fn audio_stream_index(&self) -> Option<usize> {
        self.ictx
            .streams()
            .best(ffmpeg::media::Type::Audio)
            .map(|s| s.index())
    }
}

fn copy_yuv(src: &ffmpeg::frame::Video, dst: &mut ffmpeg::frame::Video) {
    for plane in 0..3 {
        let ss = src.stride(plane);
        let ds = dst.stride(plane);
        let (h, w) = if plane == 0 {
            (src.height() as usize, src.width() as usize)
        } else {
            (src.height() as usize / 2, src.width() as usize / 2)
        };
        let sd = src.data(plane);
        let dd = dst.data_mut(plane);
        for row in 0..h {
            dd[row * ds..row * ds + w].copy_from_slice(&sd[row * ss..row * ss + w]);
        }
    }
}
