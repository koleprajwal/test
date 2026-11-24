
const noteField = document.getElementById("note");
const statusField = document.getElementById("status");
const saveButton = document.getElementById("save");
const clearButton = document.getElementById("clear");

// Load the previously saved note.
chrome.storage.local.get(["quickNote"], (result) => {
  noteField.value = result.quickNote || "";
});

// Save the note.
saveButton.addEventListener("click", () => {
  const note = noteField.value.trim();

  chrome.storage.local.set({ quickNote: note }, () => {
    if (chrome.runtime.lastError) {
      statusField.textContent = "Error saving note.";
      return;
    }

    statusField.textContent = "Note saved successfully.";
  });
});

// Remove the saved note.
clearButton.addEventListener("click", () => {
  chrome.storage.local.remove("quickNote", () => {
    if (chrome.runtime.lastError) {
      statusField.textContent = "Error clearing note.";
      return;
    }

    noteField.value = "";
    statusField.textContent = "Note cleared.";
  });
});
