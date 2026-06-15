use ffmpeg::format::Pixel;
use ffmpeg_next as ffmpeg;

pub struct VideoWriter {
    octx: ffmpeg::format::context::Output,
    encoder: ffmpeg::encoder::Video,
    out_time_base: ffmpeg::Rational,
    out_tb: ffmpeg::Rational,
    pub width: u32,
    pub height: u32,
    pts: i64,
}

impl VideoWriter {
    pub fn new(
        path: &str,
        width: u32,
        height: u32,
        fps: ffmpeg::Rational,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        // Force even dimensions (YUV420P requirement).
        let width = width & !1;
        let height = height & !1;

        let codec = ["h264_videotoolbox", "libx264"]
            .iter()
            .find_map(|n| ffmpeg::encoder::find_by_name(n))
            .or_else(|| ffmpeg::encoder::find(ffmpeg::codec::Id::H264))
            .ok_or("no H264 encoder — install libx264 or use macOS")?;

        let mut octx = ffmpeg::format::output(path)?;
        let needs_global = octx
            .format()
            .flags()
            .contains(ffmpeg::format::Flags::GLOBAL_HEADER);

        let out_time_base = ffmpeg::Rational::new(fps.1, fps.0);
        octx.add_stream(codec)?;

        let enc = {
            let mut b = ffmpeg::codec::context::Context::new_with_codec(codec)
                .encoder()
                .video()?;
            b.set_width(width);
            b.set_height(height);
            b.set_format(Pixel::YUV420P);
            b.set_time_base(out_time_base);
            b.set_frame_rate(Some(fps));
            if needs_global {
                b.set_flags(ffmpeg::codec::Flags::GLOBAL_HEADER);
            }
            b.open_as(codec)?
        };

        octx.stream_mut(0)
            .ok_or("no output stream")?
            .set_parameters(&enc);

        octx.write_header()?;
        let out_tb = octx.stream(0).unwrap().time_base();

        Ok(VideoWriter {
            octx,
            encoder: enc,
            out_time_base,
            out_tb,
            width,
            height,
            pts: 0,
        })
    }

    pub fn alloc_frame(&self) -> ffmpeg::frame::Video {
        let mut f = ffmpeg::frame::Video::new(Pixel::YUV420P, self.width, self.height);
        // Set MPEG color range explicitly to suppress the ffmpeg warning.
        f.set_color_range(ffmpeg::color::Range::MPEG);
        f
    }

    pub fn write_frame(
        &mut self,
        frame: &mut ffmpeg::frame::Video,
    ) -> Result<(), Box<dyn std::error::Error>> {
        frame.set_pts(Some(self.pts));
        self.pts += 1;
        self.encoder.send_frame(frame)?;
        self.drain_packets()?;
        Ok(())
    }

    pub fn finish(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.encoder.send_eof()?;
        self.drain_packets()?;
        self.octx.write_trailer()?;
        Ok(())
    }

    fn drain_packets(&mut self) -> Result<(), ffmpeg::Error> {
        let mut pkt = ffmpeg::Packet::empty();
        while self.encoder.receive_packet(&mut pkt).is_ok() {
            pkt.rescale_ts(self.out_time_base, self.out_tb);
            pkt.set_stream(0);
            pkt.write_interleaved(&mut self.octx)?;
        }
        Ok(())
    }
}
