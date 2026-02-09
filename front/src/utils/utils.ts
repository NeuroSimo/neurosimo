export const formatTime = (time: number | undefined): string => {
  if (time == undefined) {
    return ''
  }
  time = Math.round(time)

  const hours = Math.floor(time / 3600)
  const minutes = Math.floor((time % 3600) / 60)
  const seconds = time % 60

  let result = ''

  if (hours > 0) {
    result += `${hours} h `
  }
  if (minutes > 0) {
    result += `${minutes} min `
  }
  if (seconds > 0 || result === '') {
    result += `${seconds} s`
  }

  return result.trim()
}

export function formatFrequency(frequency: number | undefined): string {
  if (frequency === undefined || frequency === 0) {
    return '\u2013'
  }

  if (frequency < 1000) {
    return `${frequency} Hz`
  }

  const frequencyInKHz = frequency / 1000
  return `${frequencyInKHz} kHz`
}

export function formatDateTime(dateTime: string | Date, locale = 'en-US'): string {
  const date = new Date(dateTime)
  
  return new Intl.DateTimeFormat(locale, {
    year: 'numeric',
    month: 'numeric',
    day: 'numeric',
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    hour12: false
  }).format(date)
}
