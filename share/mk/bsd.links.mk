
.if !target(__<bsd.init.mk>__)
.error bsd.links.mk cannot be included directly.
.endif

.if defined(NO_ROOT)
.if !defined(TAGS) || ! ${TAGS:Mpackage=*}
TAGS+=         package=${PACKAGE}
.endif
TAG_ARGS=      -T ${TAGS:ts,:[*]}
.endif

afterinstall: _installlinks
.ORDER: realinstall _installlinks
_installlinks:
.for s t in ${LINKS}
# Darwin defaults to case folding; allow case-sensitive installation targets.
.if ${.MAKE.OS} != "Darwin" || ${INSTALL_CASE_SENSITIVE:Uno} == "yes" || ${s:tu} != ${t:tu}
.if defined(LINKTAGS)
	${INSTALL_LINK} ${TAG_ARGS:D${TAG_ARGS},${LINKTAGS}} ${DESTDIR}${s} ${DESTDIR}${t}
.else
	${INSTALL_LINK} ${TAG_ARGS} ${DESTDIR}${s} ${DESTDIR}${t}
.endif
.endif
.endfor
.for s t in ${SYMLINKS}
# Darwin defaults to case folding; allow case-sensitive installation targets.
.if ${.MAKE.OS} != "Darwin" || ${INSTALL_CASE_SENSITIVE:Uno} == "yes" || ${s:tu} != ${t:tu}
.if defined(LINKTAGS)
	${INSTALL_SYMLINK} ${TAG_ARGS:D${TAG_ARGS},${LINKTAGS}} ${s} ${DESTDIR}${t}
.else
	${INSTALL_SYMLINK} ${TAG_ARGS} ${s} ${DESTDIR}${t}
.endif
.endif
.endfor
