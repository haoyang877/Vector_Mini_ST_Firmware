/** 复用既有工作台绘图逻辑；演示位置目标单独标注，不冒充设备轨迹参考。 */
#include "scope_widget.h"
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

namespace yg::host
{
ScopeWidget::ScopeWidget(QWidget *p) : QWidget(p)
{
    setMinimumSize(360, 320);
    setMouseTracking(true);
}
void ScopeWidget::Scope_SetSamples(const std::deque<ScopeSample> &samples)
{
    if (!frozen_)
    {
        samples_ = samples;
        update();
    }
}
void ScopeWidget::Scope_SetFrozen(bool frozen)
{
    frozen_ = frozen;
    update();
}
void ScopeWidget::Scope_SetWindow(double seconds)
{
    if (std::isfinite(seconds))
    {
        seconds_ = std::clamp(seconds, 1.0, 60.0);
        update();
    }
}
void ScopeWidget::Scope_SetReplayFraction(std::optional<double> value)
{
    replay_fraction_ = value ? std::optional<double>(std::clamp(*value, 0.0, 1.0)) : std::nullopt;
    update();
}
void ScopeWidget::mouseMoveEvent(QMouseEvent *e)
{
    cursor_ = e->position().x();
    update();
}
void ScopeWidget::leaveEvent(QEvent *)
{
    cursor_ = -1;
    update();
}
void ScopeWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), QColor("#101e30"));
    const double end = samples_.empty()   ? seconds_
                       : replay_fraction_ ? samples_.front().time_s +
                                                (samples_.back().time_s - samples_.front().time_s) *
                                                    *replay_fraction_
                                          : samples_.back().time_s;
    const double start = std::max(0., end - seconds_);
    const double display_end = std::max(end, seconds_);
    const QStringList titles{"位置  /  rad", "转速  /  rad/s", "电流 Iq  /  A"};
    auto value = [](const ScopeSample &s, int channel, bool reference) -> std::optional<double>
    {
        const auto v = channel == 0   ? (reference ? s.position_reference_rad : s.position_rad)
                       : channel == 1 ? (reference ? s.speed_reference_rad_s : s.speed_rad_s)
                                      : (reference ? s.iq_reference_a : s.iq_a);
        return v && std::isfinite(*v) ? v : std::optional<double>{};
    };
    for (int ch = 0; ch < 3; ++ch)
    {
        const double panel = height() / 3.;
        const QRectF plot(68, ch * panel + 41, width() - 90, panel - 72);
        painter.setPen(QColor("#dce6f2"));
        painter.drawText(QPointF(18, ch * panel + 25), titles[ch]);
        painter.setPen(QColor("#50dacb"));
        painter.drawText(QPointF(width() - 230, ch * panel + 25), "━ 实际");
        painter.setPen(QColor("#f5bb68"));
        painter.drawText(QPointF(width() - 145, ch * panel + 25),
                         ch == 0 ? "┄ 演示目标" : "┄ 模型参考");
        double low = -1, high = 1;
        bool has_reference = false;
        for (const auto &s : samples_)
        {
            if (s.time_s >= start && s.time_s <= end)
            {
                for (bool ref : {false, true})
                {
                    auto v = value(s, ch, ref);
                    if (!v)
                    {
                        continue;
                    }
                    low = std::min(low, *v);
                    high = std::max(high, *v);
                    if (ref)
                    {
                        has_reference = true;
                    }
                }
            }
        }
        const double pad = (high - low) * .12;
        low -= pad;
        high += pad;
        auto x = [&](double t)
        {
            return plot.left() + (t - start) / (display_end - start) * plot.width();
        };
        auto y = [&](double v)
        {
            return plot.bottom() - (v - low) / (high - low) * plot.height();
        };
        for (int row = 0; row <= 4; ++row)
        {
            const double py = plot.top() + row * plot.height() / 4;
            painter.setPen(QColor("#28374a"));
            painter.drawLine(QPointF(plot.left(), py), QPointF(plot.right(), py));
            painter.setPen(QColor("#92a4b9"));
            painter.drawText(QRectF(2, py - 9, 58, 20),
                             Qt::AlignRight,
                             QString::number(high - row * (high - low) / 4, 'g', 3));
        }
        for (int col = 0; col <= 5; ++col)
        {
            const double px = plot.left() + col * plot.width() / 5;
            painter.setPen(QColor("#253447"));
            painter.drawLine(QPointF(px, plot.top()), QPointF(px, plot.bottom()));
            painter.setPen(QColor("#8296ac"));
            painter.drawText(QRectF(px - 25, plot.bottom() + 5, 52, 19),
                             Qt::AlignCenter,
                             QString::number(start + col * (display_end - start) / 5, 'f', 1) +
                                 " s");
        }
        painter.save();
        painter.setClipRect(plot.adjusted(-1, -1, 1, 1));
        for (bool reference : {true, false})
        {
            QPainterPath path;
            bool connected = false;
            double previous_time = 0;
            for (const auto &s : samples_)
            {
                if (s.time_s < start || s.time_s > end)
                {
                    continue;
                }
                auto v = value(s, ch, reference);
                if (!v)
                {
                    connected = false;
                    continue;
                }
                if (!connected || s.gap_before || s.time_s - previous_time > .2)
                {
                    path.moveTo(x(s.time_s), y(*v));
                }
                else
                {
                    path.lineTo(x(s.time_s), y(*v));
                }
                connected = true;
                previous_time = s.time_s;
            }
            painter.setPen(QPen(QColor(reference ? "#f5bb68" : "#50dacb"),
                                1.7,
                                reference ? Qt::DashLine : Qt::SolidLine));
            painter.drawPath(path);
        }
        painter.restore();
        if (!has_reference)
        {
            painter.setPen(QColor("#8e9eaf"));
            painter.drawText(
                plot.adjusted(8, 5, -8, -5), Qt::AlignTop | Qt::AlignRight, "参考未提供");
        }
        if (samples_.empty())
        {
            painter.setPen(QColor("#92a4b9"));
            painter.drawText(plot, Qt::AlignCenter, "等待采样");
        }
        if (cursor_ >= plot.left() && cursor_ <= plot.right() && !samples_.empty())
        {
            const double time =
                start + (cursor_ - plot.left()) / plot.width() * (display_end - start);
            auto nearest =
                std::min_element(samples_.begin(),
                                 samples_.end(),
                                 [time](const ScopeSample &a, const ScopeSample &b)
                                 {
                                     return std::abs(a.time_s - time) < std::abs(b.time_s - time);
                                 });
            painter.setPen(QPen(QColor("#9cb0c8"), 1, Qt::DotLine));
            painter.drawLine(QPointF(cursor_, plot.top()), QPointF(cursor_, plot.bottom()));
            painter.setPen(QColor("#e6eff7"));
            const auto v = value(*nearest, ch, false);
            const QString text = QString("t=%1 s    实际 %2")
                                     .arg(nearest->time_s, 0, 'f', 3)
                                     .arg(v ? QString::number(*v, 'f', 4) : "—");
            painter.drawText(QRectF(plot.left() + 8, plot.top() + 7, plot.width() - 16, 20),
                             Qt::AlignLeft,
                             text);
        }
    }
}
} // namespace yg::host
