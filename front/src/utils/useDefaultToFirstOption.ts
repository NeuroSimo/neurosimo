import { useEffect, useRef } from 'react'

/* Keeps a select's value valid against the options currently available.
 *
 * A <select> whose value matches none of its options displays the first option without
 * firing a change event, so the UI would show one thing while the configuration holds
 * another. That happens whenever the option list is replaced under a stored value, most
 * notably on a project switch: the draft for the new project names a file that does not
 * exist in it, or none at all.
 *
 * Selects the first available option in that case, and returns whether the value is one of
 * the options, so the caller can render a placeholder until it is.
 */
export const useDefaultToFirstOption = (
  value: string,
  options: string[],
  onSelect: (option: string) => void,
  active = true
): boolean => {
  const hasValue = value !== '' && options.includes(value)

  /* Held in a ref so that a caller passing an inline handler does not re-run the effect. */
  const onSelectRef = useRef(onSelect)
  onSelectRef.current = onSelect

  useEffect(() => {
    if (!active || hasValue || options.length === 0) return
    onSelectRef.current(options[0])
  }, [active, hasValue, options])

  return hasValue
}
