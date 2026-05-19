import React, { useRef, useEffect, useCallback } from 'react'

type StickyBottomScrollContainerProps = React.HTMLAttributes<HTMLDivElement> & {
  children: React.ReactNode
  /** When this changes, scroll to bottom only if already near the bottom */
  contentDependency?: unknown
  /** When this changes, always scroll to bottom and re-enable sticking */
  resetScrollDependency?: unknown
  bottomThresholdPx?: number
}

export const StickyBottomScrollContainer: React.FC<StickyBottomScrollContainerProps> = ({
  children,
  contentDependency,
  resetScrollDependency,
  bottomThresholdPx = 10,
  ...rest
}) => {
  const containerRef = useRef<HTMLDivElement>(null)
  const isAtBottomRef = useRef(true)

  const updateIsAtBottom = useCallback(() => {
    const el = containerRef.current
    if (!el) return
    isAtBottomRef.current =
      el.scrollHeight - el.scrollTop - el.clientHeight <= bottomThresholdPx
  }, [bottomThresholdPx])

  const scrollToBottom = useCallback(() => {
    const el = containerRef.current
    if (!el) return
    el.scrollTop = el.scrollHeight
  }, [])

  useEffect(() => {
    if (!isAtBottomRef.current) return
    scrollToBottom()
  }, [contentDependency, scrollToBottom])

  useEffect(() => {
    scrollToBottom()
    isAtBottomRef.current = true
  }, [resetScrollDependency, scrollToBottom])

  const handleScroll = (event: React.UIEvent<HTMLDivElement>) => {
    updateIsAtBottom()
    rest.onScroll?.(event)
  }

  return (
    <div {...rest} ref={containerRef} onScroll={handleScroll}>
      {children}
    </div>
  )
}
