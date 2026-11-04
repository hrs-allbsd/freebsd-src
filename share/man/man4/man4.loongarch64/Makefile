MAN=	cpuic.4 \
	eioic.4 \
	loongarch_syscon.4 \
	loongarch_timer.4 \
	ls7a_rtc.4 \
	pch_msi.4 \
	pch_pic.4

_ARCH_SUBDIR=	loongarch64
.for _manpage in ${MAN}
MLINKS+=${_manpage} ${_ARCH_SUBDIR}/${_manpage}
.endfor
MLINKS+=loongarch_timer.4 loongarch64/timer.4

.include <bsd.prog.mk>
