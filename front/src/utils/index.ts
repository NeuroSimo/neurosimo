import React from 'react'
import { targetChangeableKeys, TargetChangeableKey, Target } from 'types/target'
import {
  Pulse,
  PulseChangeableKey,
  pulseChangeableKeys,
  PulseSequenceChangeableKey,
  pulseSequenceChangeableKeys,
} from 'types/pulseSequence'

export const expand = (obj: any) =>
  obj
    ? Object.keys(obj)
        .map((key) => obj[key].toFixed(3))
        .join(', ')
    : ''

const isObject = function (obj: any) {
  return obj === Object(obj) && !Array.isArray(obj) && typeof obj !== 'function'
}
const camelToUnderscore = (key: string) => {
  return key.replace(/([A-Z])/g, '_$1').toLowerCase()
}
const snakeToCamel = (str: string) => {
  return str.replace(/([-_][a-z])/gi, ($1) => {
    return $1.toUpperCase().replace('-', '').replace('_', '')
  })
}

export const objectKeysToCamelCase = (obj: any): any => {
  if (isObject(obj)) {
    const n: any = {}

    Object.keys(obj).forEach((k) => {
      n[snakeToCamel(k)] = objectKeysToCamelCase(obj[k])
    })

    return n
  } else if (Array.isArray(obj)) {
    return obj.map((i) => {
      return objectKeysToCamelCase(i)
    })
  }

  return obj
}

export const objectKeysToSnakeCase = (obj: any): any => {
  if (isObject(obj)) {
    const n: any = {}

    Object.keys(obj).forEach((k) => {
      n[camelToUnderscore(k)] = objectKeysToSnakeCase(obj[k])
    })

    return n
  } else if (Array.isArray(obj)) {
    return obj.map((i) => {
      return objectKeysToSnakeCase(i)
    })
  }

  return obj
}

export const useFocusMemo = (): [React.RefObject<HTMLInputElement>, VoidFunction] => {
  const htmlElRef = React.useRef<HTMLInputElement>(null)
  const setFocus = React.useCallback(() => {
    if (htmlElRef.current) htmlElRef.current.focus()
  }, [htmlElRef])

  return React.useMemo(() => [htmlElRef, setFocus], [htmlElRef, setFocus])
}

export const getSequenceIndexFromRowId = (rowId: string) => Number(rowId.split('.').slice(0, -1).join('.'))

export const isOfTargetChangeableKey = (keyInput: any): keyInput is TargetChangeableKey => {
  return targetChangeableKeys.includes(keyInput)
}

export const isOfPulseSequenceChangeableKey = (keyInput: any): keyInput is PulseSequenceChangeableKey => {
  return pulseSequenceChangeableKeys.includes(keyInput)
}

export const isOfPulseChangeableKey = (keyInput: any): keyInput is PulseChangeableKey => {
  return pulseChangeableKeys.includes(keyInput)
}

export const createPulsesFromSelectedTargets = (targets: Target[]): Pulse[] => {
  return targets
    .filter((t) => t.selected)
    .map((target) => {
      return {
        targetIndex: targets.indexOf(target),
        isi: 100,
        intensity: 0.5,
        modeDuration: 100,
        visible: true,
        selected: false,
      }
    })
}

export const getKeyByValue = (object: any, value: any) => {
  if (!object) {
    return null
  }
  return Object.keys(object).find((key) => object[key] === value)
}

export const getKeyByValueExcluding = (object: any, key: string, value: any) => {
  if (!object) {
    return null
  }
  return Object.keys(object)
    .filter((k) => k !== key)
    .find((key) => object[key] === value)
}

export const getTrueKeys = (object: any) => {
  if (!object) {
    return []
  }

  const trueKeys: any = []
  for (const [key, value] of Object.entries(object)) {
    if (object[key]) {
      trueKeys.push(key)
    }
  }
  return trueKeys
}
